#!usr/bin/env python2

import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from threading import Thread, Event, Timer
from mavros_msgs.srv import CommandBool, SetMode
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from queue import Queue
import time

class Common(object):

	def __init__(self):
		self.bridge = CvBridge()
		self.pos = PoseStamped()
		self.mocap_pos = PoseStamped()
		self.state = State()

		self.gps = False
		self.ros_rate = 40
		self.alt = 4.0
		self.dead_dia_irl = 0.1016
		self.dead_switch = 0

		self.cam_alt = np.nan
		self.cv_shift = np.array([np.nan, np.nan, np.nan])
		self.led_raw = np.array([np.nan, np.nan, np.nan])
		self.led = np.array([np.nan, np.nan, np.nan])
		self.current_pose = np.array([np.nan, np.nan, np.nan, 
			np.nan, np.nan, np.nan, np.nan])
		self.takeoff_ori = np.array([np.nan, np.nan, np.nan, np.nan])
		self.cv_pose = np.array([np.nan, np.nan, np.nan])
		self.carrier_pose = np.zeros(3)

		self.local_pos_pub_docker = rospy.Publisher(
			'/docker/mavros/setpoint_position/local', PoseStamped, queue_size=1)
		self.mocap_pos_pub = rospy.Publisher(
			'/docker/mavros/mocap/pose', PoseStamped, queue_size=1)
		
		self.local_pos_sub = rospy.Subscriber(
			'/docker/mavros/local_position/pose', PoseStamped, self.pose_cb)
		self.state_docker_sub = rospy.Subscriber(
			'/docker/mavros/state', State, self.state_docker_cb)

		self.arming_client_docker = rospy.ServiceProxy(
			'/docker/mavros/cmd/arming', CommandBool)
		self.set_mode_client_docker = rospy.ServiceProxy(
			'/docker/mavros/set_mode', SetMode)

		self.pos_desired_q = Queue() 

		self.pos_pub_thread = Thread(target=self.position_pub)
		self.pos_reached_thread = Thread(target=self.position_reached)
		self.filter_thread = Thread(target=self.call_filter)
		self.center_thread = Thread(target=self.center_mav) 
		self.mocap_thread = Thread(target=self.mocap_feedback)
		self.dock_init_thread = Thread(target=self.dock_initial)
		self.dock_final_thread = Thread(target=self.dock_final)

		self.led_event = Event()
		self.filter_event = Event()
		self.center_event = Event()
		self.reached_event = Event()
		self.docking_event = Event()
		self.mocap_event = Event()
		self.final_event = Event()
		self.lost_event = Event()


	def state_docker_cb(self, data):
		self.state = data


	def pose_cb(self, data):
		self.current_pose[0] = data.pose.position.x
		self.current_pose[1] = data.pose.position.y
		self.current_pose[2] = data.pose.position.z
		self.current_pose[3] = data.pose.orientation.x
		self.current_pose[4] = data.pose.orientation.y
		self.current_pose[5] = data.pose.orientation.z
		self.current_pose[6] = data.pose.orientation.w 


	def position_reached(self, offset=1.0, gps_offset=2.0):
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			while not self.pos_desired_q.empty():
				self.reached_event.clear()
				desired_pose = self.pos_desired_q.get()
				self.gps = desired_pose[7]

				if not self.gps:
					self.pos.pose.position.x = desired_pose[0]
					self.pos.pose.position.y = desired_pose[1]
					self.pos.pose.position.z = desired_pose[2]
					self.pos.pose.orientation.x = desired_pose[3]
					self.pos.pose.orientation.y = desired_pose[4]
					self.pos.pose.orientation.z = desired_pose[5]
					self.pos.pose.orientation.w = desired_pose[6]

					reached = False
					while not reached:
						if np.linalg.norm(
							desired_pose[:3] - self.current_pose[:3]) < offset:
							time.sleep(10)
							self.reached_event.set()
							reached = True
				elif self.gps:
					self.gps_pos.latitude = desired_pose[0]
					self.gps_pos.longitude = desired_pose[1]
					self.gps_pos.altitude = desired_pose[2]

					gps_dif = np.zeros(3)
					reached = False
					while not reached:
						gps_dif[0] = 111111.0*abs(
							desired_pose[0] - self.gps_docker.latitude)
						gps_dif[1] = 111111.0*abs(
							desired_pose[1] - self.gps_docker.longitude)
						gps_dif[2] = abs(desired_pose[2] - self.gps_docker.altitude)
						if np.all(gps_dif < gps_offset):
							time.sleep(10)
							self.reached_event.set()
							reached = True

			else:
				self.center_event.set()

			rate.sleep()


	def position_setpoint(self, x, y, z, ori_x=0, ori_y=0, ori_z=0, ori_w=1, gps=False):
		self.pos_desired_q.put(np.array([x, y, z, ori_x, ori_y, ori_z, ori_w, gps]))


	def position_pub(self):
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			if not self.gps:
				self.pos.header.stamp = rospy.Time.now()
				self.local_pos_pub_docker.publish(self.pos)
			elif self.gps:
				self.gps_pos.header.stamp = rospy.Time.now()
				self.gps_docker_pub.publish(self.gps_pos)

			rate.sleep()


	def set_arm(self, arm):
		if arm and not self.state.armed:
			self.arming_client_docker(True)
			rospy.loginfo("Docker armed: %r" % arm)

		elif not arm:
			self.arming_client_docker(False)
			rospy.loginfo("Docker armed: %r" % arm)

			
	def set_offboard(self):
		if self.state.mode != "OFFBOARD":
			self.set_mode_client_docker(base_mode=0, custom_mode="OFFBOARD")
			rospy.loginfo("Docker mode: %s" % "OFFBOARD")	


	def reject_outlier(self, current, previous, offset):
		measured_offset = abs(current - previous)
		if np.all(measured_offset < offset):
			return current
		else:
			return previous


	def filter_display(self):
		rate = rospy.Rate(self.ros_rate)
		i = 0
		while not rospy.is_shutdown():
			k_gain = np.nan
			e_mea = 0.0001                            # measurement error
			e_pro = 1e-5 		   			          # process covariance
			e_est_init = np.array([0.2, 0.2, 1])      # initial estimation error
			
			if i == 0:
				led_measured = self.led_raw
				led_minus = led_measured
				e_est_minus = e_est_init + e_pro
				k_gain = e_est_minus/(e_est_minus + e_mea)
				led_kalman = led_measured
				e_est = (1 - k_gain)*led_minus

				i = 1 

			elif i == 1:
				led_measured = self.reject_outlier(self.led_raw, led_measured, 0.5)
				led_minus = led_kalman
				e_est_minus = e_est + e_pro
				k_gain = e_est_minus/(e_est_minus + e_mea)
				led_kalman = led_minus + k_gain*(led_measured - led_minus)
				e_est = (1 - k_gain)*led_minus

			self.led = led_kalman

			if not self.final_event.is_set():
				self.filter_event.set()
			else:
				self.filter_event.clear()

			rate.sleep()


	def call_filter(self):
		if self.led_event.wait():
			Timer(2.0, self.filter_display).start()


	def detect_led(self, data):
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

		# check that two contours exist (outside and inside eges of LED)
		if len(contours) < 2:
			self.lost_event.set()
			if not self.led_event.is_set():
				self.led_raw = np.array([np.nan, np.nan, np.nan])
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

			if self.filter_event.is_set():
				# extract information about the cameras view
				img_height, img_width, _ = img.shape
				img_center_x = img_width/2
				img_center_y = img_height/2
				diff_x = img_center_x - self.led[0]
				diff_y = self.led[1] - img_center_y
				fov = np.radians(80)
				foc_len = (img_width/2)/(np.tan(fov/2))

				# compute position of MAV from above values and known LED diameter
				led_dia_irl = 0.2413
				unit_dist = led_dia_irl/self.led[2]
				self.cv_pose[0] = diff_y*unit_dist
				self.cv_pose[1] = diff_x*unit_dist
				self.cv_pose[2] = foc_len*unit_dist

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


	def center_mav(self):
		if self.center_event.wait():
			self.cv_shift = self.current_pose[:3] - self.cv_pose
			self.position_setpoint(self.cv_shift[0], self.cv_shift[1], 
				self.alt + self.carrier_pose[2], self.takeoff_ori[0], 
				self.takeoff_ori[1], self.takeoff_ori[2], self.takeoff_ori[3])

			time.sleep(2)
			self.docking_event.set()


	def mocap_feedback(self):
		if self.docking_event.wait() and self.reached_event.wait():
			self.mocap_event.set()

			init_pose = self.current_pose[:3] - self.cv_pose
			self.cam_alt = self.alt - self.cv_pose[2] + self.carrier_pose[2]

			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown():
				self.mocap_pos.pose.position.x = self.cv_pose[0] + init_pose[0]
				self.mocap_pos.pose.position.y = self.cv_pose[1] + init_pose[1]
				self.mocap_pos.pose.position.z = self.cv_pose[2] + self.cam_alt

				self.mocap_pos.header.stamp = rospy.Time.now()
				self.mocap_pos_pub.publish(self.mocap_pos)

				rate.sleep()


	def dock_initial(self):
		if self.mocap_event.wait():
			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown() and not self.final_event.is_set():
				if self.reached_event.is_set():
					off_center = 2*np.sqrt(self.cv_pose[0]**2 + self.cv_pose[1]**2)
					if off_center > self.dead_dia_irl and self.dead_switch == 0:
						self.cv_shift = self.cv_shift - self.cv_pose
						self.pos.pose.position.x = self.cv_shift[0]
						self.pos.pose.position.y = self.cv_shift[1]
						self.pos.pose.position.z = 0.0
						self.pos.pose.orientation.x = self.takeoff_ori[0]
						self.pos.pose.orientation.y = self.takeoff_ori[1]
						self.pos.pose.orientation.z = self.takeoff_ori[2]
						self.pos.pose.orientation.w = self.takeoff_ori[3]
						self.dead_switch = 1
					elif off_center <= self.dead_dia_irl:
						self.dead_switch = 0

				rate.sleep()


	def dock_final(self):
		if self.mocap_event.wait():
			rate = rospy.Rate(self.ros_rate)
			while not rospy.is_shutdown() and not self.final_event.is_set():
				if self.reached_event.is_set():
					if self.cv_pose[2] >= 0.3 and self.lost_event.is_set():
						self.position_setpoint(self.cv_shift[0], 
							self.cv_shift[1], self.alt + self.carrier_pose[2], 
							self.takeoff_ori[0], self.takeoff_ori[1], 
							self.takeoff_ori[2], self.takeoff_ori[3])
						time.sleep(2)
						self.dead_switch = 0
					elif self.cv_pose[2] < 0.3 and self.dead_switch == 1:
						self.position_setpoint(self.cv_shift[0], 
							self.cv_shift[1], self.alt + self.carrier_pose[2], 
							self.takeoff_ori[0], self.takeoff_ori[1], 
							self.takeoff_ori[2], self.takeoff_ori[3])
						time.sleep(2)
						self.dead_switch = 0
					elif self.cv_pose[2] < 0.3 and self.dead_switch == 0:
						self.final_event.set()
						time.sleep(2)
						self.set_arm(False)

				rate.sleep()