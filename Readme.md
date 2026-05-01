# ASSIGNMENT 1 - Research Track 2

This project implements a ROS 2 simulation of a mobile robot in Gazebo.  
The robot can receive a target pose composed of:

- target position `x`
- target position `y`
- target orientation `theta`

and autonomously move toward the desired goal using a ROS 2 Action Server.

The simulation is started through Gazebo, while the control logic and the user interface are launched as separate ROS 2 nodes.

## Project Overview

The system is composed of three main parts:

1. **Gazebo simulation**
   - Starts the simulated world.
   - Spawns the mobile robot.
   - Provides odometry and sensor simulation.

2. **Action Server node**
   - Receives a target pose from the user.
   - Computes the pose error between the robot and the goal.
   - Publishes velocity commands on `/cmd_vel`.
   - Sends feedback during motion.
   - Returns the final robot pose once the goal is reached.
   - Allows the active goal to be canceled.

3. **UI node**
   - Allows the user to send a target pose to the robot.
   - Communicates with the Action Server through a ROS 2 action.
   - Displays feedback about the current robot pose.
   - Allows the user to cancel the active goal.
  
## Action server node
This node implements the server for the custom action SetTarget.action, which allows the user to request a new target, to receive a feedback of the execution and to cancel the current goal.
The server accepts every incoming goals and publishes the target pose as a TF frame through a static broadcaster. 
The node also subscribes to the topic /odom to obtain the odometry of the robot, which is used to publish an other TF frame.
Since both the robot frame and the goal frame are defined with respect to the world (initial position 0,0,0), at each iteration the node computes the cartesian error and
publish on the topic /cmd_vel the linear and angular velocities to reach the goal.
The implemented control strategy makes the robot follow a smooth curvilinear trajectory toward the goal.
In particula, the farther the robot is from the goal the more it is focused on reducing the distance, the closer it is the more it is focused on reaching the desired final orientation.
This type of approach is choosen in order to speed up the execution and to have a curvilinear path, instead of a simpler approach (rotation+ traslation +rotation, commented code).
The goal is considered succesfully reached when the robot reaches the position and orientation errors are below predefined thresholds.

## UI node
This node is a simple user interface which allows the user to insert the target position and orientation of the robot.
Once the user has set the goal, it is requested to the server and then executed. While executing the user receive the feedback with the current position of the robot.
Also, it is possible to cancel the current goal or to set a new goal while executing.

### TOPICS
/odom
/cmd_vel

### ACTION
SetTarget.action

### TF FRAMES
world: global reference frame
base_footprint: robot frame
goal: desired target pose

## How to run
Three terminals are required:
1. ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py
2. ros2 run assignment1_RT2 action_server_node
3. ros2 run assignment1_RT2 ui_node_exe


## Notes
The target orientation theta is provided in degrees.
The Action Server internally converts the target angle to radians.
The robot follows a direct curvilinear path toward the goal.
The goal is considered reached when the pose error is below the selected thresholds.