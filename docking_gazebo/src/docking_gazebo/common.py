import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import cv2
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from threading import Thread, Event, Timer
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from queue import Queue
import time

class Common(object):

	def __init__(self):
		# initialize ros messages
		self.bridge = CvBridge()
		self.pos = PoseStamped()
		self.vel = TwistStamped()
		self.vision_pos = PoseStamped()
		self.state = State()

		# set fixed values
		self.ros_rate = 40            # Hz
		self.alt = 5.0				  # m
		self.yaw_offset = 0.0         # deg
		self.dead_dia_irl = 0.1778	  # m
		self.shift_time = 0.0		  # s
		self.last_lost = 0.0 		  # s
		self.dead_switch = 0
		self.takeoff = True
		self.led_dia_irl = 0.2413
		self.cam_alt = 0.683514
		self.alt_thresh = 0.3
		self.descent_rate = 0.2
		self.descent_time = (self.alt_thresh + self.cam_alt)/self.descent_rate

		# initialize empty arrays, will observe errors is nans are not overwritten
		self.cv_shift = np.array([np.nan, np.nan, np.nan])
		self.current_pose = np.array([np.nan, np.nan, np.nan, 
			np.nan, np.nan, np.nan, np.nan])
		self.takeoff_ori = np.array([np.nan, np.nan, np.nan, np.nan])
		self.cv_pose_raw = np.array([np.nan, np.nan, np.nan])
		self.cv_pose = np.array([np.nan, np.nan, np.nan])
		self.cv_pose_notf = np.array([np.nan, np.nan, np.nan])
		self.carrier_pose = np.zeros(3)

		self.rolling_init = 0
		self.rolling_cv = np.zeros([2, 3])
		self.reject_time = 0
		self.collect_ind = 0
		self.collect_size = 2000
		self.raw_txt = np.empty([self.collect_size, 3])
		self.filter_txt = np.empty([self.collect_size, 3])
		self.setpoint_txt = np.empty([self.collect_size, 3])

		# ros publishers
		self.local_pos_pub_docker = rospy.Publisher(
			'/docker/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.vel_setpoint_pub = rospy.Publisher(
			'/docker/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
		self.vision_pos_pub = rospy.Publisher(
			'/docker/mavros/vision_pose/pose', PoseStamped, queue_size=1)
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
		self.collect_thread = Thread(target=self.collect_led_data)
		self.center_thread = Thread(target=self.center_mav) 
		self.vision_thread = Thread(target=self.vision_feedback)
		self.dock_init_thread = Thread(target=self.dock_initial)
		self.dock_final_thread = Thread(target=self.dock_final)
		self.vel_pub_thread = Thread(target=self.velocity_pub)
		self.vel_dock_thread = Thread(target=self.dock_velocity)

		# initialize events
		self.led_event = Event()
		self.filter_event = Event()
		self.collect_event = Event()
		self.reached_event = Event()
		self.docking_event = Event()
		self.vision_event = Event()
		self.final_event = Event()
		self.lost_event = Event()
		self.stop_pos_event = Event()


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
		while not rospy.is_shutdown() and not self.stop_pos_event.is_set():
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
				time.sleep(8)
				self.reached_event.set()
				# if there is another desired position, take it out of the queue
				if not self.pos_desired_q.empty():
					desired_pose = self.pos_desired_q.get()
					# set the desired pose
					self.set_desired_pose(desired_pose)

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
		while not rospy.is_shutdown() and not self.stop_pos_event.is_set():
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


	def rej_outlier(self, previous, current, xy_offset, z_offset):
		"""
		Compares the element wise absolute difference between two arrays and
		returns the 'previous' array if the difference to the 'current' array
		is greater than a desired 'offset' 
		"""
		offsets = abs(previous - current)
		if offsets[0] > xy_offset or offsets[1] > xy_offset or offsets[2] > z_offset:
			# reject outlier case
			return True
		else:
			return False


	def kalman_filter(self):
		"""
		Kalman filter used to filter the raw cv measurements

		Typical 1D (position only) kalman filtering method
		"""
		rate = rospy.Rate(self.ros_rate)
		switch = 0
		e_mea = 0.00001                           # measurement error
		e_pro = 1e-5 		   			          # process covariance
		e_est = np.array([0.2, 0.2, 0.4])         # initial estimation error (guess)
		cv_kalman = self.cv_pose_raw			  # initial cv pose 

		while not rospy.is_shutdown():
			cv_measured = self.cv_pose_raw
			cv_minus = cv_kalman
			e_est_minus = e_est + e_pro
			k_gain = e_est_minus/(e_est_minus + e_mea)
			cv_kalman = cv_minus + k_gain*(cv_measured - cv_minus)
			e_est = (1 - k_gain)*e_est_minus

			self.cv_pose_notf = cv_kalman

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
			Timer(1.0, self.kalman_filter).start()


	def led_to_cv(self, img, led_data):
		# extract information about the cameras view
		img_height, img_width, _ = img.shape
		img_center_x = img_width/2
		img_center_y = img_height/2
		diff_x = img_center_x - led_data[0]
		diff_y = led_data[1] - img_center_y
		fov = np.radians(80)
		foc_len = (img_width/2)/(np.tan(fov/2))

		# compute position of mav from above values and known LED diameter
		unit_dist = self.led_dia_irl/led_data[2]
		cv = np.empty(3)
		cv[0] = diff_y*unit_dist
		cv[1] = diff_x*unit_dist
		cv[2] = foc_len*unit_dist

		return cv


	def cv_to_led(self, img, cv_data):
		# extract information about the cameras view
		img_height, img_width, _ = img.shape
		img_center_x = img_width/2
		img_center_y = img_height/2
		fov = np.radians(80)
		foc_len = (img_width/2)/(np.tan(fov/2))
		
		unit_dist = cv_data[2]/foc_len
		led = np.zeros(3)
		led[0] = img_center_x - cv_data[1]/unit_dist
		led[1] = cv_data[0]/unit_dist + img_center_y
		led[2] = self.led_dia_irl/unit_dist

		return led


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
			lost_time = time.time()
			# send nan values for led if the led has not been found for the first time yet
			if not self.led_event.is_set():
				led_raw = np.array([np.nan, np.nan, np.nan])
			elif self.led_event.is_set() and lost_time - self.last_lost > 1:
				# set the event signaling that the led is not detected
				self.lost_event.set()
		# if at least two contours do exist
		else:
			# signal that vision estimates are being collected
			self.led_event.set()
			self.last_lost = time.time()
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
			led_raw = np.array([(outer_cx + inner_cx)/2, (outer_cy + inner_cy)/2, 
				(outer_dia + inner_dia)/2])

			cv_raw = self.led_to_cv(img, led_raw)

			cols = [0, 1, 2]
			if self.rolling_init < 2:
				self.rolling_cv[self.rolling_init, cols] = cv_raw
				self.cv_pose_raw = cv_raw
				self.rolling_init += 1
			else:
				self.rolling_cv[0, cols] = self.rolling_cv[1, cols]
				current_time = time.time()
				if not self.rej_outlier(self.rolling_cv[0, cols], cv_raw, 0.2, 0.4):
					self.rolling_cv[1, cols] = cv_raw
					self.reject_time = time.time()
				else:
					if current_time - self.reject_time > 1:
						self.rolling_init = 0
				self.cv_pose_raw = self.rolling_cv[1, cols]	

			if self.filter_event.is_set():
				# transform the cv pose from the camera frame to the local frame
				self.cv_pose[0] = (np.cos(self.yaw_offset)*self.cv_pose_notf[0] -
					np.sin(self.yaw_offset)*self.cv_pose_notf[1])
				self.cv_pose[1] = (np.sin(self.yaw_offset)*self.cv_pose_notf[0] +
					np.cos(self.yaw_offset)*self.cv_pose_notf[1])
				self.cv_pose[2] = self.cv_pose_notf[2]

				# display LED detection on image with a cricle and center point
				led_filtered = self.cv_to_led(img, self.cv_pose_notf)
				img = cv2.circle(img, (np.uint16(led_filtered[0]),
					np.uint16(led_filtered[1])), 
					np.uint16(led_filtered[2])/2, (0,255,0), 2)
				img = cv2.circle(img, (np.uint16(led_filtered[0]),
					np.uint16(led_filtered[1])), 2, (255,0,0), 3)

				# calculate dead zone diameter to display
				dead_led_ratio = self.dead_dia_irl/self.led_dia_irl
				dead_dia = dead_led_ratio*led_filtered[2]

				# display dead zone
				img_height, img_width, _ = img.shape
				img_center_x = img_width/2
				img_center_y = img_height/2
				img = cv2.circle(img, (np.uint16(img_center_x),np.uint16(img_center_y)), 
					np.uint16(dead_dia)/2, (0,0,0), 2)

				# display image
				cv2.imshow("Image Stream", img)
				cv2.waitKey(3)

				if self.collect_ind < self.collect_size:
					# collect raw/filtered led data into a txt file
					rows = [self.collect_ind, self.collect_ind, self.collect_ind]
					self.raw_txt[rows, cols] = \
						np.array([cv_raw[0], cv_raw[1], cv_raw[2]])
					self.filter_txt[rows, cols] = \
						np.array([self.cv_pose_notf[0], self.cv_pose_notf[1], 
						self.cv_pose_notf[2]])
					self.setpoint_txt[rows, cols] = \
						np.array([self.vel.twist.linear.x, 
						self.vel.twist.linear.y, self.vel.twist.linear.z])	
					self.collect_ind += 1
					if self.collect_ind == self.collect_size:
						self.collect_event.set()


	def collect_led_data(self):
		if self.collect_event.wait():
			print("FILTER DATA COLLECTED")
			direc = "/home/ryrocha/catkin_ws/src/docking_simulator/docking_gazebo/plots/"
			np.savetxt(direc + 
				'sim_raw_{}.csv'.format(time.strftime("%d-%b-%H:%M")), 
				self.raw_txt, delimiter=',')
			np.savetxt(direc + 
				'sim_filtered_{}.csv'.format(time.strftime("%d-%b-%H:%M")), 
				self.filter_txt, delimiter=',')
			np.savetxt(direc + 
				'sim_setpoint_{}.csv'.format(time.strftime("%d-%b-%H:%M")), 
				self.setpoint_txt, delimiter=',')


	def vision_feedback(self):
		"""
		Publishes the local position of the mav derived from the camera
		to the vision topic

		Runs in the vision_thread
		"""
		# wait for mav to reach takeoff altitude and for kalman filtering to begin
		if self.reached_event.wait() and self.filter_event.wait():
			# allow time for filter to stablize
			time.sleep(5)

			# initialize origin of cv coordinate frame in the local frame
			init_pose = self.current_pose[:3] - self.cv_pose
			self.cam_alt = self.alt - self.cv_pose[2] + self.carrier_pose[2]

			# set vision event to signal centering thread
			self.vision_event.set()

			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown():
				# add cv positions variations to initialized coordinate system
				self.vision_pos.pose.position.x = self.cv_pose[0] + init_pose[0]
				self.vision_pos.pose.position.y = self.cv_pose[1] + init_pose[1]

				# publish vision pose and update timestamp
				self.vision_pos.header.stamp = rospy.Time.now()
				self.vision_pos_pub.publish(self.vision_pos)

				rate.sleep()


	def center_mav(self):
		"""
		Centers the mav over the camera once vision feedback has been initialized
		then sets the docking_event flag to begin docking

		Runs in the center_thread
		"""
		# wait for vision to be initialized
		#if self.vision_event.wait():
		if self.reached_event.wait() and self.filter_event.wait():
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
					# if it tried correcting but has been outside the dead zone for 2s
					elif self.dead_switch == 1 and current_time - self.shift_time > 2:
						self.dead_switch = 0

					# set the desired positions	
					desired_pose = np.array([self.cv_shift[0], self.cv_shift[1], 0.0,
						self.takeoff_ori[0], self.takeoff_ori[1], 
						self.takeoff_ori[2], self.takeoff_ori[3]])
					self.set_desired_pose(desired_pose)

				rate.sleep()


	def dock_final(self):
		"""
		Once the mav is alt_thresh m above the camera, if it is within the dead zone it
		will continue docking and disarm, if it is not within the dead zone it will reset
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
					if self.cv_pose[2] >= self.alt_thresh and self.lost_event.is_set():
						self.position_setpoint(self.cv_shift[0], 
							self.cv_shift[1], self.alt + self.carrier_pose[2], 
							self.takeoff_ori[0], self.takeoff_ori[1], 
							self.takeoff_ori[2], self.takeoff_ori[3])
						time.sleep(2)
						# reset dead switch
						self.dead_switch = 0
					# if the mav is 0.4 m above the camera and is not in the dead zone
					elif self.cv_pose[2] < self.alt_thresh and self.dead_switch == 1:
						self.position_setpoint(self.cv_shift[0], 
							self.cv_shift[1], self.alt + self.carrier_pose[2], 
							self.takeoff_ori[0], self.takeoff_ori[1], 
							self.takeoff_ori[2], self.takeoff_ori[3])
						time.sleep(2)
						# reset the dead switch
						self.dead_switch = 0
					# if the mav is 0.4 m above the camera and within the dead zone
					elif self.cv_pose[2] < self.alt_thresh and self.dead_switch == 0:
						self.final_event.set()
						descent_time
						time.sleep(2)
						# signal the carreir to land
						self.carrier_land_pub.publish(True)
						# disarm the docker
						self.set_arm(False)

				rate.sleep()


	def velocity_pub(self):
		if self.stop_pos_event.wait():
			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown():
				self.vel.header.stamp = rospy.Time.now()
				self.vel_setpoint_pub.publish(self.vel)

				rate.sleep()


	def dock_velocity(self):
		if self.vision_event.wait():
		# if self.reached_event.wait() and self.filter_event.wait():
			self.stop_pos_event.set()
			rate = rospy.Rate(self.ros_rate)
			vel_ref = 0.2 
			ref_dia = 0.05
			while not rospy.is_shutdown() and not self.final_event.is_set():
				z_ref = self.alt - self.cam_alt - self.cv_pose[2]
				self.vel.twist.linear.x = -self.cv_pose[0]/ref_dia * vel_ref
				self.vel.twist.linear.y = -self.cv_pose[1]/ref_dia * vel_ref

				# calculate the offset of the mav from the center of the image
				off_center = 2*np.sqrt(self.cv_pose[0]**2 + 
					self.cv_pose[1]**2) > self.dead_dia_irl
				current_time = time.time()
				if self.cv_pose[2] >= self.alt_thresh and self.lost_event.is_set():
					self.vel.twist.linear.z = 0.5
					time.sleep(6)
				elif self.cv_pose[2] < self.alt_thresh and off_center:
					self.vel.twist.linear.z = 0.5
					time.sleep(6)
				elif self.cv_pose[2] < self.alt_thresh and not off_center:
					self.vel.twist.linear.x = 0.0
					self.vel.twist.linear.y = 0.0
					self.final_event.set()
					time.sleep(self.descent_time)
					# signal the carreir to land
					self.carrier_land_pub.publish(True)
					# disarm the docker
					self.set_arm(False)
				else:
					self.vel.twist.linear.z = -self.descent_rate
					# self.vel.twist.linear.z = z_ref/ref_dia * vel_ref
			
				rate.sleep()