# sonar_detector

## Introduction

This is a Package that provides a ROS interface for simple collision avoidance for UbiquityRobotics robots. It communicates with the ubiquity motor control node on a ROS topic to signal when the sonars are currrently seeing an object in front of the robot within a distance range that is very close to the front of the robot.

The logic implemented stops all robot drive when an object is too close to any of the 3 forward facing sonar detectors. 
By default the robot may back up and continue on it's travels or will wait till the object is no longer in the detection zone and movement may continue once again. This is a simple system meant to avoid objects that may move in front then move on out of the way.

The ROS motor node must monitor topic /system_control for defined messages that will render all cmd_vel as if they were all zero movements when in disabled mode.

## Prerequisites For This Node

This node uses are commands to the Ubiquity Robotics motor_node that are unique.  Our motor_node only recognizes the required commands on the `system_control` topic as of mid Feb of 2021. Below is how to get the current motor node.

    cd ~/catkin_ws/src
    git clone https://github.com/UbiquityRobotics/ubiquity_motor
    sudo systemctl stop magni-base
    cd ~/catkin_ws
    catkin_make
    sudo systemctl start magni-base  (or reboot later)

## Installation

This package may be installed from binaries for both x86 and ARM architectures.

The package must be manually installed and run by users who wish to implement collision avoidance in their ROS based robots.

The full source should be copied into the robot ~/catkin_ws/src/sonar_detector newly created folder and then this would form the sonar_detector node

    cd ~/catkin_ws
    catkin_make
    
## Running the Node

Once the robot has sonar board enabled and publications are active on topic `sonars` this node can be of use by starting the node manually.  This can also be done in your own launch files using standard ROS launch file techniques

To manually start this node this command can be run

    rosrun sonar_detector sonar_detector

## ROS API

### Subscribed topics

`sonars` topic is monitored where messages of type `sensor_msgs::Range` are monitored

This node assumes the Ubiquity Robotics sonar board is enabled and the `pi_sonar` node is active.

`cmd_vel` topic is monitored and by default this allows only backing up to turn off the movement disable caused by active detection of one or more objects within the collision zone


### Published topics

`system_control` is a simple string message monitored by the `motor_node` starting in early 2021

The following commands are issued to the `system_control` topic as strings

`speed_control disable`    This is sent as the node first detects a very near object in front

`speed_control enable`     This is sent as the node detects there is no longer an object very near the front.

There is some hysteresis so once we enter 'collision' range we must back away a bit to no longer be in a state of collision mode.



### Parameters

`collision_thresh_near` (double, default: 0.2)
Range in meters to center sonar sensor where we will enter collision active state

`collision_thresh_far` (double, default: 0.25)
Range in meters where when no object is in this range we exit collision active state

`enabled_sensors` (int, default: 7)
Bitfield for which sensors will be monitored for collesion detect.  Left = 1, Front = 2, Right = 4

`range_avg_factor` (double, default: 0.6)
The moving average factor weight for the current sensor reading.   A moving average is kept for each sonar being monitored.  The  average_reading = (current_reading * range_avg_factor) + (current_average * (1.0 - range_avg_factor)
You can slow down the averaging by lower numbers or 1.0 will always use most current reading, thus no averaging.

`exit_collision_on_reverse` (int, default: 1)
When set non-zero the robot is allowed to only backup once collision active state is entered.  If this were to be 0 then the robot could not move at all until the object itself moved out of range.

`loop_rate` (double, default; 25.0)
The number of times per second we look at the sonar data.  Be aware that it does not help to make this a lot faster than the rate the sonars actually are published on /sonars topic as new data must wait for what is published to /sonars topic.

