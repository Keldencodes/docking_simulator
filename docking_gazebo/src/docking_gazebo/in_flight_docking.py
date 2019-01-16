#!usr/bin/env python

import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import mavros
import cv2
from sensor_msgs.msg import Image, NavSatFix
from mavros_msgs.msg import State, ActuatorControl, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

"""
TO DO:
1) Add Kalman filter for CV position estimates and remove simple averaging
2) Use real-time to index position and velocity changes
3) Reduce motor kill distance
"""

class donut_docking:

	def __init__(self):
		# ROS/opencv interface 
		self.bridge = CvBridge()
		# image feed subscriber
		self.image_sub = rospy.Subscriber('/iris/camera/image_raw', Image, self.detect_circle)
	
		# arrays to hold cv pose data
		self.cv_pose = np.array([np.nan, np.nan, np.nan])
		self.cv_pose_avg = np.array([np.nan, np.nan, np.nan])
		# cv pose publisher (motion capture)
		self.cv_feed_pos_pub = rospy.Publisher('/donut/mavros/mocap/pose', PoseStamped, queue_size=20)

		# mav position/actuator control setpoint publishers 
		self.local_pos_pub = rospy.Publisher('/donut/mavros/setpoint_position/local', PoseStamped, queue_size=20)
		self.act_cont = rospy.Publisher('/donut/mavros/actuator_control', ActuatorControl, queue_size=20)
		self.gps_donut_pub = rospy.Publisher('/donut/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=20)
		# mav position subscribers
		self.gps_carrier_sub = rospy.Subscriber('/iris/mavros/global_position/global', NavSatFix, self.gps_cb)
		self.local_pos_sub = rospy.Subscriber('/donut/mavros/local_position/pose', PoseStamped, self.pose_cb)

		# subscriber for the current state
		self.state_sub = rospy.Subscriber('/donut/mavros/state', State, self.state_cb)
		# services for arming and setting mode requests
		self.arming_client = rospy.ServiceProxy('/donut/mavros/cmd/arming', CommandBool)
		self.set_mode_client = rospy.ServiceProxy('/donut/mavros/set_mode', SetMode) 

	def state_cb(self,data):
		# function for holding the current state of the docking mav
		global current_state
		current_state = data

	def gps_cb(self,data):
		# function for holing the gps fix of the carrier mav
		global gps_carrier
		gps_carrier = data

	def pose_cb(self,data):
		# function for holding the pose of the docking mav
		global pose_donut
		pose_donut[0] = data.pose.position.x
		pose_donut[1] = data.pose.position.y
		pose_donut[2] = data.pose.position.z

	def detect_circle(self,data):
		# main function for LED detection/processing/position estimation
		global pose_avg_store, cv_avg_poses, dead_dia_irl 
	
		# attempt to convert the raw image to opencv
		try:
			img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
		except CvBridgeError as e:
			print(e)

		# blur image edges
		img = cv2.medianBlur(img, 3)

		# convert from bgr to hsv
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	
		# threshold the HSV image, keep only the red pixels
		lower_red_hue_range = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
		upper_red_hue_range = cv2.inRange(hsv, (160, 100, 100), (179, 255, 255))
	
		# Combine the above two images
		red_hue_image = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)
		red_hue_image = cv2.GaussianBlur(red_hue_image, (9, 9), 2, 2)

		# detect contours
		_, contours, heirarchy = cv2.findContours(red_hue_image,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		# check that two contours exist (outside and inside eges of LED)
		if len(contours) < 2:
			self.cv_pose = np.array([np.nan, np.nan, np.nan])
		else:
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
			led_x = (outer_cx + inner_cx)/2
			led_y = (outer_cy + inner_cy)/2
			led_dia = (outer_dia + inner_dia)/2

			# extract information about the cameras view
			img_height, img_width, _ = img.shape
			img_center_x = img_width/2
			img_center_y = img_height/2
			diff_x = img_center_x - led_x
			diff_y = led_y - img_center_y
			fov = np.radians(80)
			foc_len = (img_width/2)/(np.tan(fov/2))
		
			# compute position of MAV from above values and known LED diameter
			led_dia_irl = 0.2413
			unit_dist = led_dia_irl/led_dia
			self.cv_pose[0] = diff_y*unit_dist
			self.cv_pose[1] = diff_x*unit_dist
			self.cv_pose[2] = foc_len*unit_dist

			# group position estimates and store averages
			if pose_avg_store >= 0:
				pose_avg_store -= 1
				cv_avg_poses[pose_avg_store,:] = self.cv_pose
			elif pose_avg_store == -1:
				pose_avg_store = 5
				self.cv_pose_avg = np.mean(cv_avg_poses, axis=0)

			# display LED detection on image with a cricle and center point
			img = cv2.circle(img, (np.uint16(led_x),np.uint16(led_y)), np.uint16(led_dia)/2, (0,255,0), 2)
			img = cv2.circle(img, (np.uint16(led_x),np.uint16(led_y)), 2, (255,0,0), 3)

			# calculate dead zone diameter to display
			dead_led_ratio = dead_dia_irl/led_dia_irl
			dead_dia = dead_led_ratio*led_dia

			# display dead zone
			img = cv2.circle(img, (np.uint16(img_center_x),np.uint16(img_center_y)), np.uint16(dead_dia)/2, (0,0,0), 2)

			# display image
			cv2.imshow("Image Stream", img)
			cv2.waitKey(3)

	def position_control(self):
		global desired_pose, pose_lag, pose_iter, dock_switch, gps_carrier, gps_switch, pose_donut, cam_alt, init_loc_pose, dead_dia_irl, dead_switch, last_update, last_desired_pose 

		# check that cv position estimates exist
		if np.isfinite(self.cv_pose).any():
			dead_rad_irl = dead_dia_irl/2
			off_cent = np.sqrt(self.cv_pose_avg[0]**2 + self.cv_pose_avg[1]**2)

			# index to delay position changes/publishing cv estimates
			if pose_lag >= 0:
				pose_lag -= 1

			# get some rough values and do initial centering
			if pose_lag == 70:
				# switch to local positioning coordinate system
				gps_switch = 0

				# estimate the z-axis location of the camera
				cam_alt = pose_donut[2] - self.cv_pose_avg[2]

				# center the docking mav on the image
				desired_pose[0] = pose_donut[0] - self.cv_pose_avg[0]
				desired_pose[1] = pose_donut[1] - self.cv_pose_avg[1]

				init_loc_pose = desired_pose

			# continue to keep docking mav within dead zone
			if pose_lag < 70:
				if off_cent > dead_rad_irl and dead_switch == 1:
					desired_pose[0] = desired_pose[0] - self.cv_pose_avg[0]
					desired_pose[1] = desired_pose[1] - self.cv_pose_avg[1]

					dead_switch = 0
				elif off_cent <= dead_rad_irl:
					dead_switch = 1 

		if gps_switch == 0:
			# desired pose to be published
			pose = PoseStamped()
			pose.pose.position.x = desired_pose[0]
			pose.pose.position.y = desired_pose[1]

			# desired motor actuation control values
			act_cont = ActuatorControl()
			act_cont.controls[0] = 0.0
			act_cont.controls[1] = 0.0
			act_cont.controls[2] = 0.0
			act_cont.controls[3] = 0.0

			# send desired poses up to a specified distance from the base
			if self.cv_pose_avg[2] > 0.4:
				# update timestamp and publish pose 
				pose.header.stamp = rospy.Time.now()
				self.local_pos_pub.publish(pose)
				last_update = rospy.get_rostime()
				last_desired_pose = desired_pose
			else:
				# desired pose to be published
				pose = PoseStamped()
				pose.pose.position.x = last_desired_pose[0]
				pose.pose.position.y = last_desired_pose[1]

				kill_motors = rospy.get_rostime()
				print(kill_motors - last_update)
				if kill_motors - last_update > rospy.Duration(3.):
					# update timestamp and kill motors
					act_cont.header.stamp = rospy.Time.now()
					self.act_cont.publish(act_cont)
				else:
					# update timestamp and publish pose
					pose.header.stamp = rospy.Time.now()
					self.local_pos_pub.publish(pose)

		elif gps_switch == 1:
			# desired pose to be published
			gps_donut = GlobalPositionTarget()
			gps_donut.latitude = gps_carrier.latitude
			gps_donut.longitude = gps_carrier.longitude
			gps_donut.altitude = gps_carrier.altitude + 6.0

			# update timestamp and publish pose 
			gps_donut.header.stamp = rospy.Time.now()
			self.gps_donut_pub.publish(gps_donut)

	def mocap_feedback(self):
		global pose_lag, pose_iter, init_loc_pose, cam_alt

		# publish cv estimates to motion capture node
		if pose_lag < 50:

			feed_pose = PoseStamped()
			feed_pose.pose.position.x = self.cv_pose_avg[0] + init_loc_pose[0]
			feed_pose.pose.position.y = self.cv_pose_avg[1] + init_loc_pose[1]
			feed_pose.pose.position.z = self.cv_pose_avg[2] + cam_alt

			# publish pose 	
			feed_pose.header.stamp = rospy.Time.now()
			self.cv_feed_pos_pub.publish(feed_pose)

# globals used primarily in keeping track of the state of the MAV
offb_set_mode = SetMode
current_state = State()
gps_carrier = NavSatFix()
pose_donut = np.array([np.nan, np.nan, np.nan])
desired_pose = np.array([np.nan, np.nan])
init_loc_pose = np.array([np.nan, np.nan, np.nan])
cam_alt = np.nan
dead_dia_irl = 0.1016
last_update = np.nan
last_desired_pose = np.array([np.nan, np.nan])

pose_lag = 140
gps_switch = 1
dead_switch = 1

pose_avg_store = 5
cv_avg_poses = np.empty((pose_avg_store,3))

# main function for exectuing donut_docking node
def main():
	# call donut_docking class and initialize node
	dd = donut_docking()
	rospy.init_node('donut_docking')

	global current_state
	prev_state = current_state

	# data rate (must be more than 2Hz)
	rate = rospy.Rate(7.0)

	# send a few position setpoints before starting
	for i in range(100):
		dd.position_control()
		rate.sleep()
    
	# wait for FCU connection
	while not current_state.connected:
		rate.sleep()

	# arm the MAV and set to OFFBOARD mode
	last_request = rospy.get_rostime()
	while not rospy.is_shutdown():
		now = rospy.get_rostime()
		if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
			dd.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
			last_request = now 
		else:
			if not current_state.armed and (now - last_request > rospy.Duration(5.)):
				dd.arming_client(True)
				last_request = now 

		# log MAV state and mode changes
		if prev_state.armed != current_state.armed:
			rospy.loginfo("Vehicle armed: %r" % current_state.armed)
		if prev_state.mode != current_state.mode:
			rospy.loginfo("Current mode: %s" % current_state.mode)
		prev_state = current_state
		
		# call position_control functions to update timestamp and publish pose  
		dd.position_control()
		dd.mocap_feedback()
		rate.sleep()

if __name__ == '__main__':
	main()