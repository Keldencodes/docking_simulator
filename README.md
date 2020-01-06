Autonomous In-flight Multi-rotor Docking Simulator
===============

![](sim_img.jpg)

This is a Gazebo simulation to demonstrate an approach to autonomous in-flight docking of unmanned multi-rotor aerial vehicles (MAV). The results of the simulation can be seen [here](https://youtu.be/kJOZesGr-7w).

**Simulation Dependencies:**

- Ubuntu 16.04
- ROS Kinetic
- Gazebo 7
- Python 2.7
- OpenCV 3
- PX4 Firmware
- RotorS

A proper installation of Ubuntu 16.04 will include Python 2.7. Note that the algorithms used in this simulation require the NumPy library as well. This [link](https://www.learnopencv.com/install-opencv3-on-ubuntu/) is useful for properly installing OpenCV 3. The installation procedure for RotorS can be followed [here](https://github.com/ethz-asl/rotors_simulator). The procedures [here](https://dev.px4.io/v1.8.2/en/setup/dev_env_linux.html) should be followed for installing PX4/ROS Kinetic/Gazebo and building the PX4 Firmware's SITL environment.

**Running the Simulation**
1. Clone the repository: `git clone https://github.com/ryrocha/docking_simulator.git`
2. Build the simulator packages:
```
cd ~/catkin_ws
catkin build docking_description
catkin build docking_gazebo
```
3. Launch the simulation: 
- For the stationary docking simulation: `roslaunch docking_gazebo stationary_base_docking.launch` 
- For the in-flight docking simulation: `roslaunch docking_gazebo in_flight_docking.launch` 
4. Start the ROS nodes used to control the drones:
- For the stationary docking simulation: `rosrun docking_gazebo stationary_docking`
- For the in-flight docking simulation: In one terminal window `rosrun docking_gazebo in_flight_docking` followed by `rosrun docking_gazebo carrier_control` in another
