#!usr/bin/env python2

import roslib
roslib.load_manifest('docking_gazebo')
from common import Common
from threading import Timer
import rospy
import time
from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix, Image
from geometry_msgs.msg import PoseStamped


class in_flight_docking(Common):

	def __init__(self):
		super(in_flight_docking, self).__init__()

		self.gps_carrier = NavSatFix()
		self.gps_docker = NavSatFix()
		self.gps_pos = GlobalPositionTarget()

		self.gps_docker_pub = rospy.Publisher(
			'/donut/mavros/setpoint_position/global', GlobalPositionTarget, queue_size=1)

		self.gps_carrier_sub = rospy.Subscriber(
			'/iris/mavros/global_position/global', NavSatFix, self.gps_carrier_cb)
		self.gps_docker_sub = rospy.Subscriber(
			'/donut/mavros/global_position/global', NavSatFix, self.gps_docker_cb)
		self.image_sub = rospy.Subscriber(
			'/iris/camera/image_raw', Image, self.detect_led)
		self.carrier_pos_sub = rospy.Subscriber(
			'/iris/mavros/local_position/pose', PoseStamped, self.carrier_pose_cb)


	def gps_carrier_cb(self, data):
		self.gps_carrier = data


	def gps_docker_cb(self, data):
		self.gps_docker = data


	def carrier_pose_cb(self, data):
		self.carrier_pose[0] = data.pose.position.x
		self.carrier_pose[1] = data.pose.position.y
		self.carrier_pose[2] = data.pose.position.z


	def docking_procedure(self):
		# delay arming
		time.sleep(5)

		# arm the MAV
		self.set_arm(True)

		self.alt = 5.0
		self.gps_alt = self.gps_carrier.altitude + self.alt

		# send takeoff setpoint
		self.position_setpoint(self.gps_docker.latitude, self.gps_docker.longitude, 
			self.gps_alt, gps=True)

		# start threads to publish positions and check that positions are being reached
		self.pos_reached_thread.start()
		self.pos_pub_thread.start()

		# delay switch to offboard mode to ensure sufficient initial setpoint stream
		Timer(5.0, self.set_offboard).start()

		# bring within vision range
		self.position_setpoint(self.gps_carrier.latitude, self.gps_carrier.longitude, 
			self.gps_alt, gps=True)

		# begin filtering vision data
		self.filter_thread.start()

		# center the mav on the image
		self.center_thread.start()

		# begin motion capture feedback
		self.mocap_thread.start()

		# dock 
		self.dock_init_thread.start()
		self.dock_final_thread.start()


def main():
	# initialize ros node 
	rospy.init_node('in_flight_docking')

	# begin the docking procedure 
	in_flight_docking().docking_procedure()

if __name__ == '__main__':
	main()