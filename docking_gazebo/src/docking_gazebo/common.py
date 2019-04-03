#!usr/bin/env python2

import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import cv2
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from threading import Thread, Event, Timer, Condition
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from queue import Queue

class Common(object):

	def __init__(self):
		self.pos = PoseStamped()

		self.local_pos_pub = rospy.Publisher(
			'/donut/mavros/setpoint_position/local', PoseStamped, queue_size=20)

		self.pos_pub_thread = Thread(target=self.position_pub)

		self.state_sub = rospy.Subscriber('/donut/mavros/state', State, self.state_cb)
		self.state = State()

		self.arming_client = rospy.ServiceProxy('/donut/mavros/cmd/arming', CommandBool)

		self.set_mode_client = rospy.ServiceProxy('/donut/mavros/set_mode', SetMode) 

		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber(
			'/base/camera1/image_raw', Image, self.detect_led)
		self.led_raw = np.array([np.nan, np.nan, np.nan])
		self.led_event = Event()
		self.led = np.array([np.nan, np.nan, np.nan])
		self.filter_thread = Thread(target=self.call_filter)
		self.cv_pose = np.array([np.nan, np.nan, np.nan])
		self.filter_event = Event()

	def state_cb(self, data):
		self.state = data

	def position_setpoint(self, x, y, z):
		

	def position_pub(self):
		rate = rospy.Rate(8)

		while not rospy.is_shutdown():
			self.pos.header.stamp = rospy.Time.now()
			self.local_pos_pub.publish(self.pos)

	def set_arm(self, arm):
		if arm and not self.state.armed:
			self.arming_client(True)
			rospy.loginfo("Vehicle armed: %r" % arm)

		elif not arm:
			self.arming_client(False)
			rospy.loginfo("Vehicle armed: %r" % arm)
			
	def set_offboard(self):
		if self.state.mode != "OFFBOARD":
			self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
			rospy.loginfo("Current mode: %s" % "OFFBOARD")	

	def reject_outlier(self, current, previous, offset):
		measured_offset = abs(current - previous)
		if np.all(measured_offset < offset):
			return current
		else:
			return previous

	def filter_display(self):
		rate = rospy.Rate(8)

		i = 0
		while not rospy.is_shutdown():
			k_gain = np.nan
			e_mea = 10.0                              # measurement error
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

			self.filter_event.set()

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
			if not self.led_event.is_set():
				self.led_raw = np.array([np.nan, np.nan, np.nan])
		else:
			# signal that vision estimates are being collected
			self.led_event.set()

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
				dead_dia_irl = 0.1016
				dead_led_ratio = dead_dia_irl/led_dia_irl
				dead_dia = dead_led_ratio*self.led[2]

				# display dead zone
				img = cv2.circle(img, (np.uint16(img_center_x),np.uint16(img_center_y)), 
					np.uint16(dead_dia)/2, (0,0,0), 2)

				# display image
				cv2.imshow("Image Stream", img)
				cv2.waitKey(3)