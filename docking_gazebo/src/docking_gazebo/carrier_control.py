#!usr/bin/env python2

import roslib
roslib.load_manifest('docking_gazebo')
from common import Common
from threading import Thread, Timer
import rospy
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class carrier_control(Common):

	def __init__(self):
		super(carrier_control, self).__init__()

		self.carrier_pos = PoseStamped()
		self.carrier_state = State()

		self.local_pos_pub_carrier = rospy.Publisher(
			'/iris/mavros/setpoint_position/local', PoseStamped, queue_size=1)

		self.arming_client_carrier = rospy.ServiceProxy(
			'/iris/mavros/cmd/arming', CommandBool)
		self.set_mode_client_carrier = rospy.ServiceProxy(
			'/iris/mavros/set_mode', SetMode)

		self.carrier_pos_thread = Thread(target=self.carrier_pos_pub, args=(0, 0, 2,))


	def state_carrier_cb(self, data):
		self.carrier_state = data


	def carrier_pos_pub(self, x, y, z):
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			self.carrier_pos.pose.position.x = x
			self.carrier_pos.pose.position.y = y
			self.carrier_pos.pose.position.z = z

			self.carrier_pos.header.stamp = rospy.Time.now()
			self.local_pos_pub_carrier.publish(self.carrier_pos)

			rate.sleep()


	def carrier_procedure(self):
		# delay arming
		time.sleep(5)

		# arm the MAV
		if not self.carrier_state.armed:
			self.arming_client_carrier(True)
			rospy.loginfo("Carrier armed: %r" % True)

		# send takeoff setpoint
		self.carrier_pos_thread.start()

		# delay switch to offboard mode to ensure sufficient initial setpoint stream
		time.sleep(5)
		if self.carrier_state.mode != "OFFBOARD":
			self.set_mode_client_carrier(base_mode=0, custom_mode="OFFBOARD")
			rospy.loginfo("Carrier mode: %s" % "OFFBOARD")


def main():
	# initialize ros node 
	rospy.init_node('carrier_control')

	# begin the docking procedure 
	carrier_control().carrier_procedure()

if __name__ == '__main__':
	main()