#!usr/bin/env python

import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import mavros
import cv2
from sensor_msgs.msg import Image
from mavros_msgs.msg import State, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

"""
TO DO:
1) Add Kalman filter to CV position estimates
2) Use real-time to index position and velocity changes
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
		# mav position/actuator control setpoint publisher 
		self.local_pos_pub = rospy.Publisher('/donut/mavros/setpoint_position/local', PoseStamped, queue_size=20)
		self.act_cont = rospy.Publisher('/donut/mavros/actuator_control', ActuatorControl, queue_size=20)
	
		# subscriber for the current state
		self.state_sub = rospy.Subscriber('/donut/mavros/state', State, self.state_cb)
		# services for arming and setting mode requests
		self.arming_client = rospy.ServiceProxy('/donut/mavros/cmd/arming', CommandBool)
		self.set_mode_client = rospy.ServiceProxy('/donut/mavros/set_mode', SetMode) 

	def state_cb(self,data):
		# function for holding the current state when the state subscriber is called
		global current_state
		current_state = data

	def detect_circle(self,data):
		# main function for LED detection/processing/position estimation
		global pose_avg_store, cv_avg_poses
	
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

			# display image
			cv2.imshow("Image Stream", img)
			cv2.waitKey(3)

	def position_control(self):
		global desired_pose, pose_lag, pose_iter, dock_switch
		
		# check that cv position estimates exist
		if np.isfinite(self.cv_pose).any():
			# delay position changes/publishing cv estimates
			if pose_lag >= 0:
				pose_lag -= 1
			# shift position of MAV to center of image 
			elif pose_lag == -1 and pose_iter == 1:
				pose_lag = 70
				pose_iter -= 1

				cv_shift = np.subtract(np.array([0.0, 0.0, 3.0]), self.cv_pose_avg)
				desired_pose = np.add(desired_pose, cv_shift)
			# correct MAV position once mocap is engaged
			elif pose_lag == -70 and pose_iter == 0:
				cv_shift = np.subtract(np.array([0.0, 0.0, 3.0]), self.cv_pose_avg)
				desired_pose = np.add(desired_pose, cv_shift)

				dock_switch = 1

		if dock_switch == 0:
			# desired pose to be published
			pose = PoseStamped()
			pose.pose.position.x = desired_pose[0]
			pose.pose.position.y = desired_pose[1]
			pose.pose.position.z = desired_pose[2]

			# update timestamp and publish pose 
			pose.header.stamp = rospy.Time.now()
			self.local_pos_pub.publish(pose)
		elif dock_switch == 1:
			# desired pose to be published
			pose = PoseStamped()
			pose.pose.position.x = desired_pose[0]
			pose.pose.position.y = desired_pose[1]

			# kill motors
			act_cont = ActuatorControl()
			act_cont.controls[0] = 0.0
			act_cont.controls[1] = 0.0
			act_cont.controls[2] = 0.0
			act_cont.controls[3] = 0.0

			# send desired poses up to a specified distance from the base
			if self.cv_pose_avg[2] > 0.5:
				# update timestamp and publish pose 
				pose.header.stamp = rospy.Time.now()
				self.local_pos_pub.publish(pose)
			else:
				# update timestamp and kill motors
				act_cont.header.stamp = rospy.Time.now()
				self.act_cont.publish(act_cont)

	def mocap_feedback(self):
		global pose_lag, pose_iter

		# publish cv estimates to motion capture node
		if pose_lag <= -1 and pose_iter == 0:
			pose_lag -= 1

			feed_pose = PoseStamped()
			feed_pose.pose.position.x = self.cv_pose_avg[0] - 3.0
			feed_pose.pose.position.y = self.cv_pose_avg[1] - 5.0
			feed_pose.pose.position.z = self.cv_pose_avg[2] + 1.683514

			# publish pose 	
			feed_pose.header.stamp = rospy.Time.now()
			self.cv_feed_pos_pub.publish(feed_pose)

# globals used primarily in keeping track of the state of the MAV
offb_set_mode = SetMode
current_state = State()

desired_pose = np.array([-1.0, -4.0, 6.0])
pose_lag = 70
pose_iter = 1
dock_switch = 0

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
		
		# call position_control function to update timestamp and publish pose  
		dd.position_control()
		dd.mocap_feedback()
		rate.sleep()

if __name__ == '__main__':
	main()