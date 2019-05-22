#!usr/bin/env python2

import roslib
roslib.load_manifest('docking_gazebo')
from common import Common
from threading import Thread, Event
import rospy
import time
from mavros_msgs.msg import State
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, GlobalPositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix
from queue import Queue
import numpy as np


class carrier_control(Common):

	def __init__(self):
		super(carrier_control, self).__init__()

		self.carrier_alt = 2.0 
		self.carrier_land_bool = False
		self.carrier_pos = PoseStamped()
		self.carrier_state = State()
		self.initial_setpoint = NavSatFix()
		self.gps_carrier = NavSatFix()
		self.gps_docker = NavSatFix()
		self.gps_pos = GlobalPositionTarget()

		self.gps_carrier_pub = rospy.Publisher(
			'/carrier/mavros/setpoint_position/global', 
			GlobalPositionTarget, queue_size=1)

		self.state_carrier_sub = rospy.Subscriber(
			'/carrier/mavros/state', State, self.state_carrier_cb)
		self.gps_carrier_sub = rospy.Subscriber(
			'/carrier/mavros/global_position/global', NavSatFix, self.gps_carrier_cb)
		self.gps_docker_sub = rospy.Subscriber(
			'/docker/mavros/global_position/raw/fix', NavSatFix, self.gps_docker_cb)
		self.carrier_land_sub = rospy.Subscriber(
			'/carrier/land_bool', Bool, self.carrier_land_cb)

		self.arming_client_carrier = rospy.ServiceProxy(
			'/carrier/mavros/cmd/arming', CommandBool)
		self.set_mode_client_carrier = rospy.ServiceProxy(
			'/carrier/mavros/set_mode', SetMode)

		self.carrier_desired_q = Queue()

		self.carrier_pub_thread = Thread(target=self.carrier_pub)
		self.carrier_reached_thread = Thread(target=self.carrier_reached)
		self.carrier_land_thread = Thread(target=self.carrier_land)

		self.carrier_reached_event = Event()


	def state_carrier_cb(self, data):
		self.carrier_state = data


	def gps_carrier_cb(self, data):
		self.gps_carrier = data


	def gps_docker_cb(self, data):
		self.gps_docker = data


	def carrier_land_cb(self, data):
		self.carrier_land_bool = data.data
		

	def carrier_reached(self, gps_offset=2.0):
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			while not self.carrier_desired_q.empty():
				self.carrier_reached_event.clear()
				desired_pose = self.carrier_desired_q.get()

				self.gps_pos.latitude = desired_pose[0]
				self.gps_pos.longitude = desired_pose[1]
				self.gps_pos.altitude = desired_pose[2]	

				gps_dif = np.zeros(3)
				reached = False
				while not reached:
					gps_dif[0] = 111111.0*abs(
						desired_pose[0] - self.gps_carrier.latitude)
					gps_dif[1] = 111111.0*abs(
						desired_pose[1] - self.gps_carrier.longitude)
					gps_dif[2] = abs(desired_pose[2] - self.gps_carrier.altitude)
					if np.all(gps_dif < gps_offset):
						time.sleep(10)
						self.carrier_reached_event.set()
						reached = True	


	def carrier_setpoint(self, x, y, z):
		self.carrier_desired_q.put(np.array([x, y, z]))	


	def carrier_pub(self):
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			self.gps_pos.header.stamp = rospy.Time.now()
			self.gps_carrier_pub.publish(self.gps_pos)

			rate.sleep()


	def carrier_arm(self, arm):
		if arm and not self.carrier_state.armed:
			self.arming_client_carrier(True)
			rospy.loginfo("Carrier armed: %r" % arm)

		elif not arm:
			self.arming_client_carrier(False)
			rospy.loginfo("Carrier armed: %r" % arm)


	def carrier_land(self):
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			if self.carrier_land_bool:
				self.carrier_setpoint(self.gps_docker.latitude, self.gps_docker.longitude,
					self.initial_setpoint.altitude)
				time.sleep(5)
				self.carrier_arm(False)

				break

			rate.sleep()	


	def carrier_procedure(self):
		# delay arming
		time.sleep(5)

		self.initial_setpoint = self.gps_carrier

		# arm the MAV
		self.carrier_arm(True)

		# send desired setpoints
		self.carrier_setpoint(self.gps_carrier.latitude, self.gps_carrier.longitude, 
			self.initial_setpoint.altitude + 2.0)
		self.carrier_setpoint(self.gps_docker.latitude, self.gps_docker.longitude,
			self.initial_setpoint.altitude + 2.0)

		# start publisher, reacher and landing thread
		self.carrier_reached_thread.start()
		self.carrier_pub_thread.start()
		self.carrier_land_thread.start()

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