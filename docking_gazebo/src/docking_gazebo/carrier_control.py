#!usr/bin/env python

import roslib
roslib.load_manifest('docking_gazebo')
import rospy
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
import numpy as np


class carrier_control:

	def __init__(self):
		# mav position/actuator control setpoint publisher 
		self.local_pos_pub = rospy.Publisher('/iris/mavros/setpoint_position/local', PoseStamped, queue_size=20)
	
		# subscriber for the current state
		self.state_sub = rospy.Subscriber('/iris/mavros/state', State, self.state_cb)
		# services for arming and setting mode requests
		self.arming_client = rospy.ServiceProxy('/iris/mavros/cmd/arming', CommandBool)
		self.set_mode_client = rospy.ServiceProxy('/iris/mavros/set_mode', SetMode) 

	def state_cb(self,data):
		# function for holding the current state when the state subscriber is called
		global current_state
		current_state = data

	def position_control(self):
		# desired pose to be published
		pose = PoseStamped()
		pose.pose.position.x = 0.0
		pose.pose.position.y = 0.0
		pose.pose.position.z = 1.0

		# update timestamp and publish pose 
		pose.header.stamp = rospy.Time.now()
		self.local_pos_pub.publish(pose)

# globals used primarily in keeping track of the state of the MAV
current_state = State()

# main function for exectuing carrier_control node
def main():
	# call carrier_control class and initialize node
	cc = carrier_control()
	rospy.init_node('carrier_control')

	global current_state
	prev_state = current_state

	# data rate (must be more than 2Hz)
	rate = rospy.Rate(7.0)

	# send a few position setpoints before starting
	for i in range(100):
		cc.position_control()
		rate.sleep()
    
	# wait for FCU connection
	while not current_state.connected:
		rate.sleep()

	# arm the MAV and set to OFFBOARD mode
	last_request = rospy.get_rostime()
	while not rospy.is_shutdown():
		now = rospy.get_rostime()
		if current_state.mode != "OFFBOARD" and (now - last_request > rospy.Duration(5.)):
			cc.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
			last_request = now 
		else:
			if not current_state.armed and (now - last_request > rospy.Duration(5.)):
				cc.arming_client(True)
				last_request = now 

		# log MAV state and mode changes
		if prev_state.armed != current_state.armed:
			rospy.loginfo("Vehicle armed: %r" % current_state.armed)
		if prev_state.mode != current_state.mode:
			rospy.loginfo("Current mode: %s" % current_state.mode)
		prev_state = current_state
		
		# call position_control function to update timestamp and publish pose  
		cc.position_control()
		rate.sleep()

if __name__ == '__main__':
	main()