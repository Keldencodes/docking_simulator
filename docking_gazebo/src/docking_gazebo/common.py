#!usr/bin/env python2

import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from threading import Thread, Event, Timer
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from queue import Queue
import time
import tf

class Common(object):

	def __init__(self):
		# initialize ros messages
		self.bridge = CvBridge()
		self.pos = PoseStamped()
		self.mocap_pos = PoseStamped()
		self.state = State()

		# set fixed values
		self.ros_rate = 40            # Hz
		self.alt = 4.0				  # m
		self.dead_dia_irl = 0.1016	  # m
		self.shift_time = 0.0		  # s
		self.dead_switch = 0
		self.takeoff = True

		# initialize empty arrays, will observe errors is nans are not overwritten
		self.cam_alt = np.nan
		self.cv_shift = np.array([np.nan, np.nan, np.nan])
		self.led_raw = np.array([np.nan, np.nan, np.nan])
		self.led = np.array([np.nan, np.nan, np.nan])
		self.current_pose = np.array([np.nan, np.nan, np.nan, 
			np.nan, np.nan, np.nan, np.nan])
		self.takeoff_ori = np.array([np.nan, np.nan, np.nan, np.nan])
		self.cv_pose = np.array([np.nan, np.nan, np.nan])
		self.cv_pose_raw = np.array([np.nan, np.nan, np.nan])
		self.carrier_pose = np.zeros(3)

		# ros publishers
		self.local_pos_pub_docker = rospy.Publisher(
			'/docker/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.mocap_pos_pub = rospy.Publisher(
			'/docker/mavros/mocap/pose', PoseStamped, queue_size=1)
		self.carrier_land_pub = rospy.Publisher(
			'/carrier/land_bool', Bool, queue_size=1)
		
		# ros subscribers
		self.local_pos_sub = rospy.Subscriber(
			'/docker/mavros/local_position/pose', PoseStamped, self.pose_cb)
		self.state_docker_sub = rospy.Subscriber(
			'/docker/mavros/state', State, self.state_docker_cb)

		# ros service proxies
		self.arming_client_docker = rospy.ServiceProxy(
			'/docker/mavros/cmd/arming', CommandBool)
		self.set_mode_client_docker = rospy.ServiceProxy(
			'/docker/mavros/set_mode', SetMode)

		# initialize queue to hold position setpoints
		self.pos_desired_q = Queue() 

		# initialize threads 
		self.pos_pub_thread = Thread(target=self.position_pub)
		self.pos_reached_thread = Thread(target=self.position_reached)
		self.filter_thread = Thread(target=self.call_filter)
		self.center_thread = Thread(target=self.center_mav) 
		self.mocap_thread = Thread(target=self.mocap_feedback)
		self.dock_init_thread = Thread(target=self.dock_initial)
		self.dock_final_thread = Thread(target=self.dock_final)

		# initialize events
		self.led_event = Event()
		self.filter_event = Event()
		self.reached_event = Event()
		self.docking_event = Event()
		self.mocap_event = Event()
		self.final_event = Event()
		self.lost_event = Event()


	def state_docker_cb(self, data):
		"""
		Callback function for state of the docker

		Pertains directly to the state_docker_sub subscriber
		data - includes information (arming, mode, etc.) pertaining to the state
		of the FCU on the docker mav in the State() format
		"""
		self.state = data


	def pose_cb(self, data):
		"""
		Callback function for the local position of the docker

		Pertains directly to the local_pos_sub subscriber
		data - includes the position and orientation of the docker mav 
		in the local frame in ENU coordinates in the PoseStamped() format
		"""
		self.current_pose[0] = data.pose.position.x
		self.current_pose[1] = data.pose.position.y
		self.current_pose[2] = data.pose.position.z
		self.current_pose[3] = data.pose.orientation.x
		self.current_pose[4] = data.pose.orientation.y
		self.current_pose[5] = data.pose.orientation.z
		self.current_pose[6] = data.pose.orientation.w 


	def set_desired_pose(self, desired_pose):
		"""
		Function takes in pose data from an array and populates a PoseStamped message
		"""
		self.pos.pose.position.x = desired_pose[0]
		self.pos.pose.position.y = desired_pose[1]
		self.pos.pose.position.z = desired_pose[2]
		self.pos.pose.orientation.x = desired_pose[3]
		self.pos.pose.orientation.y = desired_pose[4]
		self.pos.pose.orientation.z = desired_pose[5]
		self.pos.pose.orientation.w = desired_pose[6]


	def position_reached(self, offset=1.0):
		"""
		Checks if a position setpoint is reached

		Runs in the pos_reached_thread and is called whenever the position queue
		is given a new position setpoint
		"""
		desired_pose = np.zeros(6)
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			# check if mav is at desired position
			off_check = np.linalg.norm(desired_pose[:3] - self.current_pose[:3]) < offset
			# check it position has been reached
			reached = self.reached_event.is_set()
			# mav will always start in a takeoff state
			if self.takeoff:
				# extract the position setpoint from the queue
				desired_pose = self.pos_desired_q.get()
				# set the desired pose
				self.set_desired_pose(desired_pose)
				self.takeoff = False
			elif off_check and not reached:
				# if there is another desired position, take it out of the queue
				if not self.pos_desired_q.empty():
					desired_pose = self.pos_desired_q.get()
					# set the desired pose
					self.set_desired_pose(desired_pose)

				time.sleep(10)
				self.reached_event.set()
			elif not off_check and not self.docking_event.is_set():
				self.reached_event.clear()
			elif reached and not self.pos_desired_q.empty():
				desired_pose = self.pos_desired_q.get()
				# set the desired pose
				self.set_desired_pose(desired_pose)
				self.reached_event.clear()

			rate.sleep()


	def position_setpoint(self, x, y, z, ori_x=0, ori_y=0, ori_z=0, ori_w=1):
		"""
		Takes a desired position and orientation setpoint and adds them to a queue
		"""
		self.pos_desired_q.put(np.array([x, y, z, ori_x, ori_y, ori_z, ori_w]))


	def position_pub(self):
		"""
		Publishes position/orientation setpoints

		Runs in the pos_pub_thread to continously publish setpoints which is 
		a requirement for offboard mode
		"""
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			self.pos.header.stamp = rospy.Time.now()
			self.local_pos_pub_docker.publish(self.pos)

			rate.sleep()


	def set_arm(self, arm):
		"""
		Arms and disarms (based on imput) the docker mav and prints the arm state
		"""
		if arm and not self.state.armed:
			self.arming_client_docker(True)
			rospy.loginfo("Docker armed: %r" % arm)

		elif not arm:
			self.arming_client_docker(False)
			rospy.loginfo("Docker armed: %r" % arm)

			
	def set_offboard(self):
		"""
		Switches the flight mode of the FCU on the docker to offboard 
		"""
		if self.state.mode != "OFFBOARD":
			self.set_mode_client_docker(base_mode=0, custom_mode="OFFBOARD")
			rospy.loginfo("Docker mode: %s" % "OFFBOARD")	


	def reject_outlier(self, current, previous, offset):
		"""
		Compares the element wise absolute difference between two arrays and
		returns the 'previous' array if the difference to the 'current' array
		is greater than a desired 'offset' 
		"""
		measured_offset = abs(current - previous)
		if np.all(measured_offset < offset):
			return current
		else:
			return previous


	def filter_display(self):
		"""
		Kalman filter used to filter the raw cv measurements

		Typical 1D (position only) kalman filtering method
		"""
		rate = rospy.Rate(self.ros_rate)
		i = 0
		while not rospy.is_shutdown():
			k_gain = np.nan
			e_mea = 0.01                            # measurement error
			e_pro = 1e-5 		   			          # process covariance
			e_est_init = np.array([0.2, 0.2, 1])      # initial estimation error
			
			# initial pass of kalman filter
			if i == 0:
				led_measured = self.led_raw
				led_minus = led_measured
				e_est_minus = e_est_init + e_pro
				k_gain = e_est_minus/(e_est_minus + e_mea)
				led_kalman = led_measured
				e_est = (1 - k_gain)*led_minus

				i = 1 

			# main kalman filter loop
			elif i == 1:
				"""fully reject measurements if they fall outside of 0.2 m in one timestep
				the mav does not move fast enough to jump 0.2 m in a timestep 
				this error can be attributed to loss of proper contour detection
				we really only want to filter the valid cv data"""
				led_measured = self.reject_outlier(self.led_raw, led_measured, 0.2)
				led_minus = led_kalman
				e_est_minus = e_est + e_pro
				k_gain = e_est_minus/(e_est_minus + e_mea)
				led_kalman = led_minus + k_gain*(led_measured - led_minus)
				e_est = (1 - k_gain)*led_minus

			self.led = led_kalman

			# set the filter event if has not been set and the mav is not docked
			if not self.filter_event.is_set() and not self.final_event.is_set():
				self.filter_event.set()
			# clear the filter event if the mav docks, this stops cv pose updates
			elif self.final_event.is_set():
				self.filter_event.clear()

			rate.sleep()


	def call_filter(self):
		"""
		Calls the kalman filter shortly after led detection begins

		Runs in the filter_thread
		"""
		if self.led_event.wait():
			Timer(5.0, self.filter_display).start()


	def detect_led(self, data):
		"""
		Callback function to process the video stream from the camera

		data - the individual frames from the camera stream
		"""
		try:
			img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)

		# blur image edges
		img = cv2.medianBlur(img, 3)

		# convert from bgr to hsv
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		
		# threshold the HSV image, keep only the red pixels
		lower_red_hue_range = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255))
		upper_red_hue_range = cv2.inRange(hsv, (160, 100, 50), (179, 255, 255))
		
		# Combine the above two images
		red_hue_image = cv2.bitwise_or(lower_red_hue_range, upper_red_hue_range)
		red_hue_image = cv2.GaussianBlur(red_hue_image, (9, 9), 2, 2)

		# detect contours
		_, contours, _ = cv2.findContours(
			red_hue_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		# if two contours are not detected
		if len(contours) < 2:
			# set the event signaling that the led is not detected
			self.lost_event.set()
			# send nan values for led if the led has not been found for the first time yet
			if not self.led_event.is_set():
				self.led_raw = np.array([np.nan, np.nan, np.nan])
		# if at least two contours do exist
		else:
			# signal that vision estimates are being collected
			self.led_event.set()
			self.lost_event.clear()

			# extract the outer and inner edges of the LED
			outer_cnt = contours[0]
			inner_cnt = contours[1]

			# compute the equivalent diameters of the inner/outer edges
			outer_area = cv2.contourArea(outer_cnt)
			inner_area = cv2.contourArea(inner_cnt)
			outer_dia = np.sqrt(4.0*outer_area/np.pi)
			inner_dia = np.sqrt(4.0*inner_area/np.pi)

			# find the centers of the inner/outer edges
			outer_M = cv2.moments(outer_cnt)
			inner_M = cv2.moments(inner_cnt)
			outer_cx = outer_M['m10']/outer_M['m00']
			outer_cy = outer_M['m01']/outer_M['m00']
			inner_cx = inner_M['m10']/inner_M['m00']
			inner_cy = inner_M['m01']/inner_M['m00']

			# take the averages of the values calculated above
			self.led_raw[0] = (outer_cx + inner_cx)/2
			self.led_raw[1] = (outer_cy + inner_cy)/2
			self.led_raw[2] = (outer_dia + inner_dia)/2

			# if the led measurements are being filtered
			if self.filter_event.is_set():
				# extract information about the cameras view
				img_height, img_width, _ = img.shape
				img_center_x = img_width/2
				img_center_y = img_height/2
				diff_x = img_center_x - self.led[0]
				diff_y = self.led[1] - img_center_y
				fov = np.radians(80)
				foc_len = (img_width/2)/(np.tan(fov/2))

				# compute position of mav from above values and known LED diameter
				led_dia_irl = 0.2413
				unit_dist = led_dia_irl/self.led[2]
				self.cv_pose_raw[0] = diff_y*unit_dist
				self.cv_pose_raw[1] = diff_x*unit_dist
				self.cv_pose_raw[2] = foc_len*unit_dist

				# transform the cv pose from the camera frame to the local frame
				yaw_offset = tf.transformations.euler_from_quaternion(self.takeoff_ori)[2]
				self.cv_pose[0] = (np.cos(yaw_offset)*self.cv_pose_raw[0] -
					np.sin(yaw_offset)*self.cv_pose_raw[1])
				self.cv_pose[1] = (np.sin(yaw_offset)*self.cv_pose_raw[0] +
					np.cos(yaw_offset)*self.cv_pose_raw[1])
				self.cv_pose[2] = self.cv_pose_raw[2]

				# display LED detection on image with a cricle and center point
				img = cv2.circle(img, (np.uint16(self.led[0]),np.uint16(self.led[1])), 
					np.uint16(self.led[2])/2, (0,255,0), 2)
				img = cv2.circle(img, 
					(np.uint16(self.led[0]),np.uint16(self.led[1])), 2, (255,0,0), 3)

				# calculate dead zone diameter to display
				dead_led_ratio = self.dead_dia_irl/led_dia_irl
				dead_dia = dead_led_ratio*self.led[2]

				# display dead zone
				img = cv2.circle(img, (np.uint16(img_center_x),np.uint16(img_center_y)), 
					np.uint16(dead_dia)/2, (0,0,0), 2)

				# display image
				cv2.imshow("Image Stream", img)
				cv2.waitKey(3)


	def mocap_feedback(self):
		"""
		Publishes the local position of the mav 
		derived from the camera to the motion capture topic

		Runs in the mocap_thread
		"""
		# wait for mav to reach takeoff altitude and for kalman filtering to begin
		if self.reached_event.wait() and self.filter_event.wait():
			# allow time for filter to get through initial values
			time.sleep(5)

			# initialize origin of cv coordinate frame in the local frame
			init_pose = self.current_pose[:3] - self.cv_pose
			self.cam_alt = self.alt - self.cv_pose[2] + self.carrier_pose[2]

			# set motion capture event to signal centering thread
			self.mocap_event.set()

			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown():
				# add cv positions variations to initialized coordinate system
				self.mocap_pos.pose.position.x = self.cv_pose[0] + init_pose[0]
				self.mocap_pos.pose.position.y = self.cv_pose[1] + init_pose[1]
				self.mocap_pos.pose.position.z = self.cv_pose[2] + self.cam_alt

				# publish mocap pose and update timestamp
				self.mocap_pos.header.stamp = rospy.Time.now()
				self.mocap_pos_pub.publish(self.mocap_pos)

				rate.sleep()


	def center_mav(self):
		"""
		Centers the mav over the camera once motion capture has been initialized
		then sets the docking_event flag to begin docking

		Runs in the center_thread
		"""
		# wait for mocap to be initialized
		if self.mocap_event.wait():
			# cv_shift is the pose correction of the mav to center it over the camera
			self.cv_shift = self.current_pose[:3] - self.cv_pose
			# set new centered position
			self.position_setpoint(self.cv_shift[0], self.cv_shift[1], 
				self.alt + self.carrier_pose[2], self.takeoff_ori[0], 
				self.takeoff_ori[1], self.takeoff_ori[2], self.takeoff_ori[3])

			# delay docking event to ensure reached event is cleared
			time.sleep(2)
			self.docking_event.set()


	def dock_initial(self):
		"""
		Once the mav is centered, this function is called to begin the descent of the
		mav as well as keep it centered on the image within the limits of a 'dead zone'.
		If it falls outside of the 'dead zone' a centering correction command is sent.

		Runs in the dock_init_thread 
		"""
		if self.docking_event.wait():
			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown() and not self.final_event.is_set():
				# wait for centering to be finished
				if self.reached_event.is_set():
					# time to be compared to time at which a centering correction is made
					current_time = time.time()

					# calculate the offset of the mav from the center of the image
					off_center = 2*np.sqrt(self.cv_pose[0]**2 + self.cv_pose[1]**2)
					# if it is outside of the dead zone and has not corrected itself
					if off_center > self.dead_dia_irl and self.dead_switch == 0:
						self.cv_shift = self.cv_shift - self.cv_pose
						self.dead_switch = 1
						# time of centering correction
						self.shift_time = time.time()
					# if it is within the dead zone
					elif off_center <= self.dead_dia_irl:
						self.dead_switch = 0
					# if it tried correcting but has been in the dead zone for 2 secs
					elif self.dead_switch == 1 and current_time - self.shift_time > 2.0:
						self.dead_switch = 0

					# set the desired positions	
					desired_pose = np.array([self.cv_shift[0], self.cv_shift[1], 0.0,
						self.takeoff_ori[0], self.takeoff_ori[1], 
						self.takeoff_ori[2], self.takeoff_ori[3]])
					self.set_desired_pose(desired_pose)

				rate.sleep()


	def dock_final(self):
		"""
		Once the mav is 0.3 m above the camera, if it is within the dead zone it will 
		continue docking and disarm, if it is not within the dead zone it will reset
		itself by going to the starting altitude and repeating the initial docking 
		procedure. If at any point during the initial docking procedure the camera
		looses led detection it will reset itself.

		Runs in the dock_final_thread
		"""
		if self.docking_event.wait():
			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown() and not self.final_event.is_set():
				# make sure it waits when a position setpoint is given
				if self.reached_event.is_set():
					# if at any point the led is not detected
					if self.cv_pose[2] >= 0.3 and self.lost_event.is_set():
						self.position_setpoint(self.cv_shift[0], 
							self.cv_shift[1], self.alt + self.carrier_pose[2], 
							self.takeoff_ori[0], self.takeoff_ori[1], 
							self.takeoff_ori[2], self.takeoff_ori[3])
						time.sleep(2)
						# reset dead switch
						self.dead_switch = 0
					# if the mav is 0.3 m above the camera and is not in the dead zone
					elif self.cv_pose[2] < 0.3 and self.dead_switch == 1:
						self.position_setpoint(self.cv_shift[0], 
							self.cv_shift[1], self.alt + self.carrier_pose[2], 
							self.takeoff_ori[0], self.takeoff_ori[1], 
							self.takeoff_ori[2], self.takeoff_ori[3])
						time.sleep(2)
						# reset the dead switch
						self.dead_switch = 0
					# if the mav is 0.3 m above the camera and within the dead zone
					elif self.cv_pose[2] < 0.3 and self.dead_switch == 0:
						self.final_event.set()
						time.sleep(2)
						# signal the carreir to land
						self.carrier_land_pub.publish(True)
						# disarm the docker
						self.set_arm(False)

				rate.sleep()