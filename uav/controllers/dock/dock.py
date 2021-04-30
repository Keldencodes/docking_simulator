#!/usr/bin/env python3.8

"""Docker controller. Version 4/30/21"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Keyboard
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit

import math


def CLAMP(value, low, high):
	if value < low:
		return low
	elif value > high:
		return high
	return value


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

print("Let's try to set all the motors and instruments\n")
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
frontLeft = robot.getDevice('front left propeller')
frontRight = robot.getDevice('front right propeller')
rearLeft = robot.getDevice('rear left propeller')
rearRight = robot.getDevice('rear right propeller')
motors = [frontLeft, frontRight, rearLeft, rearRight]

for m in range(4):
	motors[m].setPosition(math.inf)
	motors[m].setVelocity(1)

print("Motors set!\n")

imu = robot.getDevice('inertial unit')
imu.enable(timestep)

gps = robot.getDevice('gps')
gps.enable(timestep)

gyro = robot.getDevice('gyro')
gyro.enable(timestep)

kb = Keyboard()
try: 
	kb.enable(timestep)
except:
	print("keyboard fail")

print("Instruments and control initiated!\n")

while robot.step(timestep) != -1:
	if robot.getTime() > 1.0:
		break

# Display manual control message.
print("You can control the drone with your computer keyboard:\n")
print("- 'up': move forward.\n")
print("- 'down': move backward.\n")
print("- 'right': turn right.\n")
print("- 'left': turn left.\n")
print("- 'shift + up': increase the target altitude.\n")
print("- 'shift + down': decrease the target altitude.\n")
print("- 'shift + right': strafe right.\n")
print("- 'shift + left': strafe left.\n")

# Constants, empirically found.
k_vertical_thrust = 80   # 68.5  # with this thrust, drone lifts.
# 0.6  # Vert offset where robot actually targets to stabilize itself.
k_vertical_offset = 1
k_vertical_p = 3.0        # P constant of the vertical PID.
k_roll_p = 50.0           # P constant of the roll PID.
k_pitch_p = 30.0          # P constant of the pitch PID.

# Variables.
target_altitude = 1.0  # The target altitude. Can be changed by the user.

roll_disturbance = 0.0
pitch_disturbance = 0.0
yaw_disturbance = 0.0


def kbUp():
	global pitch_disturbance
	pitch_disturbance = 2.0
	print(pitch_disturbance)


def kbDown():
	global pitch_disturbance
	pitch_disturbance = -2.0
	print(pitch_disturbance)



def kbRight():
	global yaw_disturbance
	yaw_disturbance = 1.3
	print(yaw_disturbance)


def kbLeft():
	global yaw_disturbance
	yaw_disturbance = -1.3
	print(yaw_disturbance)


def kbShiftRight():
	global roll_disturbance
	roll_disturbance = -2.0
	print(roll_disturbance)


def kbShiftLeft():
	global roll_disturbance
	roll_disturbance = 2.0
	print(roll_disturbance)


def kbShiftUp():
	global target_altitude
	target_altitude += 0.04
	print("target altitude: %.2f [m]\n" %target_altitude)


def kbShiftDown():
	global target_altitude
	target_altitude -= 0.04
	print("target altitude: %.2f [m]\n" %target_altitude)


switcher = {
	# found these values by printing key value to console
	314: kbLeft,
	315: kbUp,
	316: kbRight,
	317: kbDown,
	65850: kbShiftLeft,
	65851: kbShiftUp,
	65852: kbShiftRight,
	65853: kbShiftDown
}

print("ok here we go!\n")
# Main loop:
# - perform simulation steps until Webots stops the controller
while robot.step(timestep) != -1:

	time = robot.getTime()  # in seconds

	roll = imu.getRollPitchYaw()[0] + math.pi/2
	pitch = imu.getRollPitchYaw()[1]
	roll_acceleration = gyro.getValues()[0]
	pitch_acceleration = gyro.getValues()[1]

	altitude = gps.getValues()[1]

	# Transform the keyboard input to disturbances on the stabilization algorithm.
	roll_disturbance = 0.0
	pitch_disturbance = 0.0
	yaw_disturbance = 0.0
	key = kb.getKey()
	while key > 0:
		if key == 314:
			yaw_disturbance = -1.3
		elif key == 315:
			pitch_disturbance = 2.0
		elif key == 316:
			yaw_disturbance = 1.3
		elif key == 317:
			pitch_disturbance = -2.0
		elif key == 65850:
			roll_disturbance = 2.0
		elif key == 65851:
			target_altitude += 0.04
			print("target altitude: %.2f [m]\n" %target_altitude)
		elif key == 65852:
			roll_disturbance = -2.0
		elif key == 65853:
			target_altitude -= 0.04
			print("target altitude: %.2f [m]\n" %target_altitude)
		
		#switcher.get(key, lambda: "Invalid key")
		key = kb.getKey()

	# Compute the roll, pitch, yaw and vertical inputs.
	roll_input = k_roll_p * CLAMP(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
	pitch_input = k_pitch_p * CLAMP(pitch, -1.0, 1.0) - pitch_acceleration + pitch_disturbance
	yaw_input = yaw_disturbance
	clamped_difference_altitude = CLAMP(target_altitude - altitude + k_vertical_offset, -1.0, 1.0)
	vertical_input = k_vertical_p * pow(clamped_difference_altitude, 3.0)

	# Actuate the motors taking into consideration all the computed inputs.
	front_left_motor_input = k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
	front_right_motor_input = k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
	rear_left_motor_input = k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
	rear_right_motor_input = k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
	frontLeft.setVelocity(front_left_motor_input)
	frontRight.setVelocity(-front_right_motor_input)
	rearLeft.setVelocity(-rear_left_motor_input)  
	rearRight.setVelocity(rear_right_motor_input)  


# Enter here exit cleanup code.
