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

		# set fixed values
		self.carrier_alt = 4.0 	# m
		self.carrier_land_bool = False
		self.carrier_takeoff = True

		# initialize ros messages
		self.carrier_pos = PoseStamped()
		self.carrier_state = State()
		self.initial_setpoint = NavSatFix()
		self.gps_carrier = NavSatFix()
		self.gps_docker = NavSatFix()
		self.gps_pos = GlobalPositionTarget()

		# ros publishers
		self.gps_carrier_pub = rospy.Publisher(
			'/carrier/mavros/setpoint_position/global', 
			GlobalPositionTarget, queue_size=1)

		# ros subscribers
		self.state_carrier_sub = rospy.Subscriber(
			'/carrier/mavros/state', State, self.state_carrier_cb)
		self.gps_carrier_sub = rospy.Subscriber(
			'/carrier/mavros/global_position/global', NavSatFix, self.gps_carrier_cb)
		self.gps_docker_sub = rospy.Subscriber(
			'/docker/mavros/global_position/raw/fix', NavSatFix, self.gps_docker_cb)
		self.carrier_land_sub = rospy.Subscriber(
			'/carrier/land_bool', Bool, self.carrier_land_cb)

		# ros service proxies
		self.arming_client_carrier = rospy.ServiceProxy(
			'/carrier/mavros/cmd/arming', CommandBool)
		self.set_mode_client_carrier = rospy.ServiceProxy(
			'/carrier/mavros/set_mode', SetMode)

		# initialize queue to hold position setpoints
		self.carrier_desired_q = Queue()

		# initialize threads
		self.carrier_pub_thread = Thread(target=self.carrier_pub)
		self.carrier_reached_thread = Thread(target=self.carrier_reached)
		self.carrier_land_thread = Thread(target=self.carrier_land)

		# initialize events
		self.carrier_reached_event = Event()


	def state_carrier_cb(self, data):
		"""
		Callback function for state of the carrier

		Pertains directly to the state_carrier_sub subscriber
		data - includes information (arming, mode, etc.) pertaining to the state
		of the FCU on the carrier mav in the State() format
		"""
		self.carrier_state = data


	def gps_carrier_cb(self, data):
		"""
		Callback function for the global position of the carrier

		Pertains directly to the gps_carrier_sub subscriber
		data - includes the gps coordinates of the carrier mav 
		in the global frame in NED coordinates in the NavSatFix() format
		"""
		self.gps_carrier = data


	def gps_docker_cb(self, data):
		"""
		Callback function for the global position of the docker

		Pertains directly to the gps_docker_sub subscriber
		data - includes the gps coordinates of the docker mav 
		in the global frame in NED coordinates in the NavSatFix() format
		"""
		self.gps_docker = data


	def carrier_land_cb(self, data):
		"""
		Callback function for the boolean pertaining to desired landing state

		True when carrier needs to land and False when it does not 
		"""
		self.carrier_land_bool = data.data
		

	def set_carrier_gps(self, desired_pose):
		self.gps_pos.latitude = desired_pose[0]
		self.gps_pos.longitude = desired_pose[1]
		self.gps_pos.altitude = desired_pose[2]


	def carrier_reached(self, gps_offset=1.0):
		"""
		Checks if a position setpoint is reached

		Runs in the carrier_reached_thread and is called whenever the position queue
		is given a new position setpoint
		"""
		gps_dif = np.zeros(3)
		desired_pose = np.zeros(6)
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			# latitude offset from desired of carrier
			gps_dif[0] = 111111.0*abs(desired_pose[0] - self.gps_carrier.latitude)
			# longitude offset from desired of carrier
			gps_dif[1] = 111111.0*abs(desired_pose[1] - self.gps_carrier.longitude)
			# altitude offset from desired of carrier
			gps_dif[2] = abs(desired_pose[2] - self.gps_carrier.altitude)
			# check if mav is as desired position
			carrier_off_check = np.all(gps_dif < gps_offset)

			# check it position has been reached
			carrier_reached = self.carrier_reached_event.is_set()

			if self.carrier_takeoff:
				# extract the position setpoint from the queue
				desired_pose = self.carrier_desired_q.get()
				# set the desired pose
				self.set_carrier_gps(desired_pose)
				self.carrier_takeoff = False
			elif carrier_off_check and not carrier_reached:
				time.sleep(5)
				self.carrier_reached_event.set()
				# if there is another desired position, take it out of the queue
				if not self.carrier_desired_q.empty():
					desired_pose = self.carrier_desired_q.get()

					# set the desired pose
					self.set_carrier_gps(desired_pose)

			elif not carrier_off_check:
				self.carrier_reached_event.clear()
			elif carrier_reached and not self.carrier_desired_q.empty():
				desired_pose = self.carrier_desired_q.get()
				# set the desired pose
				self.set_carrier_gps(desired_pose)
				self.carrier_reached_event.clear()

			rate.sleep()


	def carrier_setpoint(self, x, y, z):
		"""
		Takes a desired position and orientation setpoint and adds them to a queue
		"""
		self.carrier_desired_q.put(np.array([x, y, z]))	


	def carrier_pub(self):
		"""
		Publishes position/orientation setpoints

		Runs in the carrier_pub_thread to continously publish setpoints which is 
		a requirement for offboard mode
		"""
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			self.gps_pos.header.stamp = rospy.Time.now()
			self.gps_carrier_pub.publish(self.gps_pos)

			rate.sleep()


	def carrier_arm(self, arm):
		"""
		Arms and disarms (based on imput) the carrier mav and prints the arm state
		"""
		if arm and not self.carrier_state.armed:
			self.arming_client_carrier(True)
			rospy.loginfo("Carrier armed: %r" % arm)

		elif not arm:
			self.arming_client_carrier(False)
			rospy.loginfo("Carrier armed: %r" % arm)


	def carrier_land(self):
		"""
		Sets the altitude of the carrier to zero and disarms it after 5 seconds
		"""
		rate = rospy.Rate(self.ros_rate)
		while not rospy.is_shutdown():
			# check if it is time for landing depending if the docker has finished docking
			if self.carrier_land_bool:
				time.sleep(10)
				self.carrier_setpoint(self.gps_docker.latitude, self.gps_docker.longitude,
					self.initial_setpoint.altitude - 10.0)
				time.sleep(15)
				# disarm the carrier
				self.carrier_arm(False)

				break

			rate.sleep()	


	def carrier_procedure(self):
		# delay arming
		time.sleep(5)

		# collect takeoff position
		self.initial_setpoint = self.gps_carrier

		# arm the mav
		self.carrier_arm(True)

		# send desired setpoints
		self.carrier_setpoint(self.gps_carrier.latitude, self.gps_carrier.longitude, 
			self.initial_setpoint.altitude + 4.0)
		self.carrier_setpoint(self.gps_docker.latitude, self.gps_docker.longitude,
			self.initial_setpoint.altitude + 4.0)

		# delay switch to offboard mode to ensure sufficient initial setpoint stream
		time.sleep(5)
		if self.carrier_state.mode != "OFFBOARD":
			self.set_mode_client_carrier(base_mode=0, custom_mode="OFFBOARD")
			rospy.loginfo("Carrier mode: %s" % "OFFBOARD")

		# start position publisher and reacher threads
		self.carrier_reached_thread.start()
		self.carrier_pub_thread.start()

		# start thread to land the carrier 
		self.carrier_land_thread.start()


def main():
	# initialize ros node 
	rospy.init_node('carrier_control')

	# begin the docking procedure 
	carrier_control().carrier_procedure()

if __name__ == '__main__':
	main()