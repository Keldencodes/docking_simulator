#!/usr/bin/env python3.8

"""Docker controller. This is outdated, DO NOT USE. Instead: uav/controllers/dock/dock.py"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Keyboard
from controller import Compass
from controller import GPS
from controller import Gyro
from controller import InertialUnit

import math

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

print("Let's try to set all the motors and instruments\n")
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
frontLeft = robot.getMotor('front left propeller')
frontRight = robot.getMotor('front right propeller')
rearLeft = robot.getMotor('rear left propeller')
rearRight = robot.getMotor('rear right propeller')
motors = [frontLeft, frontRight, rearLeft, rearRight]

for m in motors:
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
kb.enable(timestep)

print("Instruments and control initiated!\n")

while robot.step(timestep) != -1:
    if robot.getTime()> 1.0:
      break

# Display manual control message.
print("You can control the drone with your computer keyboard:\n");
print("- 'up': move forward.\n");
print("- 'down': move backward.\n");
print("- 'right': turn right.\n");
print("- 'left': turn left.\n");
print("- 'shift + up': increase the target altitude.\n");
print("- 'shift + down': decrease the target altitude.\n");
print("- 'shift + right': strafe right.\n");
print("- 'shift + left': strafe left.\n");

# Constants, empirically found.
k_vertical_thrust = 80   # 68.5  # with this thrust, drone lifts.
k_vertical_offset = 1    # 0.6  # Vert offset where robot actually targets to stabilize itself.
k_vertical_p = 3.0        # P constant of the vertical PID.
k_roll_p = 50.0           # P constant of the roll PID.
k_pitch_p = 30.0          # P constant of the pitch PID.

# Variables.
target_altitude = 1.0;  # The target altitude. Can be changed by the user.

roll_disturbance = 0.0;
pitch_disturbance = 0.0;
yaw_disturbance = 0.0;

def kbUp():
	pitch_disturbance = 2.0
	
def kbDown():
	pitch_disturbance = -2.0

def kbRight():
	yaw_disturbance = 1.3

def kbLeft():
	yaw_disturbance = -1.3

def kbShiftRight():
	roll_disturbance = -2.0

def kbShiftLeft():
	roll_disturbance = 2.0

def kbShiftUp():
	target_altitude += 0.04

def kbShiftDown():
	target_altitude -= 0.04



switcher = {
	WB_KEYBOARD_UP: pitch_disturbance = 2.0

	WB_KEYBOARD_DOWN: pitch_disturbance = -2.0

	WB_KEYBOARD_RIGHT:
          yaw_disturbance = 1.3

	WB_KEYBOARD_LEFT:
          yaw_disturbance = -1.3

	(WB_KEYBOARD_SHIFT + WB_KEYBOARD_RIGHT): roll_disturbance = -1.0

	(WB_KEYBOARD_SHIFT + WB_KEYBOARD_LEFT): roll_disturbance = 1.0

	(WB_KEYBOARD_SHIFT + WB_KEYBOARD_UP): target_altitude += 0.05
          print("target altitude: %f [m]\n", target_altitude)

	(WB_KEYBOARD_SHIFT + WB_KEYBOARD_DOWN): target_altitude -= 0.05
          print("target altitude: %f [m]\n", target_altitude)

      }

# Main loop:
# - perform simulation steps until Webots stops the controller
while robot.step(timestep) != -1:
	
	print("ok here we go!\n")
	
	time = robot.getTime()
	
	roll = imu.getRollPitchYaw()[0] + math.pi/2 
    pitch = imu.getRollPitchYaw()[1]
    roll_acceleration = gyro.getValues()[0] 
    pitch_acceleration = gyro.getValues()[1]
    
    altitude = gps.getValues()[1]
    
    
    # Transform the keyboard input to disturbances on the stabilization algorithm.

    key = kb.getKey()
    while key > 0:
      
      key = wb_keyboard_get_key();
    }
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
