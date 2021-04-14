# sonar_detector

## Introduction

This is a Package that provides a ROS interface for simple collision avoidance for UbiquityRobotics robots. It communicates with the ubiquity motor control node on a ROS topic to signal when the sonars are currrently seeing an object in front of the robot within a distance range that is very close to the front of the robot.

The logic implemented stops all robot drive when an object is too close to any of the 3 forward facing sonar detectors. 
By default the robot may back up and continue on it's travels or will wait till the object is no longer in the detection zone and movement may continue once again. This is a simple system meant to avoid objects that may move in front then move on out of the way.

Note that this node does not do any sort of route re-planning or other navigation functions.  It just avoids hitting things that the sonar units can detect.  The 3 sonar units used each detect objects that are within a +/- 25 degree cone of detection that is out in front of each sonar board.  Objects that are to the side or above or below that cone of detection will not be 'seen'

The current ROS motor_node monitors the topic /system_control for defined messages that will render all cmd_vel as if they were all zero movements when in disabled mode.

The sonar_detector node is not designed to be used while move_basic is in use.  That is because move_basic already has it's own collision avoidance so using this node at the same time of move_basic should not be done.

We have near completion another simple node that uses sonars to 'wander around a room' on it's own and that will at some future date be in our demos repository under demos/sonar_wanderer.

## Prerequisites For This Node

None of the code is on any image as of March 2021.   The ubiquity_motor code of the future will have the ability to recognize the messages on topic /system_control at some point in the future.

There are no plans for the sonar_detector to be shipped by default on our images so you must get it from our github

### Current Version Of the ubiquity_motor node

This node uses are commands to the Ubiquity Robotics motor_node that are unique.  Our motor_node only recognizes the required commands on the `system_control` topic as of mid Feb of 2021. Below is how to get the current motor node.

If you already have the folder of ~/catkin_ws/src/ubiquity_motor then do this:

    cd ~/catkin_ws/src/ubiquity_motor
    git pull

If you do not have the above folder then do this:

    cd ~/catkin_ws/src
    git clone https://github.com/UbiquityRobotics/ubiquity_motor

You now have the source code and can next get sonar_detector code.
You will have to do a make once both ubiquity_motor as well as the sonar_detector code are in your catkin_workspace

### Installation of the sonar_detector code

The package must be manually installed and run by users who wish to implement collision avoidance in their ROS based robots.
There are no existing images with sonar_detector and it is considered an optional package

    cd ~/catkin_ws/src
    git clone https://github.com/UbiquityRobotics/sonar_detector

### Do a make so the code can be used

    cd ~/catkin_ws
    sudo systemctl stop magni-base
    catkin_make
    sudo shutdown -r now

After the robot reboots you may run the node at any time in the future use of the robot
    
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

