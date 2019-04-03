#!usr/bin/env python2

import roslib
roslib.load_manifest('docking_gazebo')
from common import Common
from threading import Timer
import rospy

class stationary_docking(Common):

	def __init__(self):
		super(stationary_docking, self).__init__()

	def docking_procedure(self):
		# arm the MAV
		self.set_arm(True)

		# start position publishing thread and send takeoff setpoint
		self.position_setpoint(0, 0, 6)
		self.pos_pub_thread.start()

		# delay switch to offboard mode to ensure sufficient initial setpoint stream
		Timer(5.0, self.set_offboard).start()

		# begin filtering vision data
		self.filter_thread.start()

		# change the mav position
		self.position_setpoint(-1, 0, 4)
		self.position_setpoint(0, -1, 5)
		self.position_setpoint(0, 0, 6)

def main():
	# initialize ros node 
	rospy.init_node('stationary_docking')

	# begin the docking procedure 
	stationary_docking().docking_procedure()

if __name__ == '__main__':
	main()