/*
 * Copyright (c) 2021, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

/*
 *
 * Implements a ROS node that uses Magni Robot Sonar data to detect collision possibility
 *
 * This node sends override control to the motor_node on a topic that the motor node 
 * monitors.  This node will disable motor node from moving when object is too close.
 * Optionally the node will allow reverse operation to back away from the collision and work again.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/Range.h"
#include <geometry_msgs/Twist.h>

/**
 * Monitor sonar sensor data and running average of selected sensors to detect near collision
 */
#define COL_THRESH_NEAR   ((double)(0.2))     // When we drop into detect mode
#define COL_THRESH_FAR    ((double)(0.25))    // When we drop out of detection noted
#define SYS_CONTROL_TOPIC "system_control"    // ROS topic recognized by motor controller
#define DETECTOR_LOOP_RATE  25                // This sets loops per second
#define RANGE_AVG_FACTOR  ((double)(0.6))     // Latest sample weight for running average 

// Values that indicate which sensors are to be used or have seen object too close
#define SENSOR_LEFT   1
#define SENSOR_FRONT  2
#define SENSOR_RIGHT  4
#define ENABLED_SENSORS  (SENSOR_LEFT|SENSOR_FRONT|SENSOR_RIGHT)


// A class to use Magni sonar range sensors to detect frontal collision risk
class SonarDetector {
  private:
    ros::Subscriber sonarSub;
    ros::Subscriber cmdVelSub;

    ros::Publisher sysControlPub;

    double loop_rate;               // Rate at which we sample sonar ranges

    // Hold the averaged distances for each sensor we support
    double dist_front;
    double dist_right;
    double dist_left;

    int    enabled_sensors;        // We allow for detection of zero or any combination of sensors

    double forwardVel;
    double angularVel;
    int    zeroMotorSpeed;
    int    collisionBits;

    int    exit_on_rev_mode;      // If non-zero we will exit collision mode when velocity is reverse

    // Define values that we decide we have an object in collision range
    double col_thresh_near;
    double col_thresh_far;

    double range_avg_factor;     // The moving average factor.  Weight of current sample out of 1.0

    void sonarCallback(const sensor_msgs::Range::ConstPtr& msg);
    void cmdVelCallback(const ::geometry_msgs::Twist::ConstPtr& msg);

  public:
    SonarDetector();
    void run(void);
    int  collisionDetect(double dist_front);   // Detects collision based in dist from front passed in
    int  isCollision();                        // Indicates of collision is likely with forward movement
};

/**
 * Callback function executes when new sonar topic data comes.
 * We keep a running average of each sensor being monitored
 */
void SonarDetector::sonarCallback(const sensor_msgs::Range::ConstPtr& msg)
{
  std::string sonarId(msg->header.frame_id);
  double sonarRange = msg->range;

  ROS_DEBUG("Sonar '%s' Seq: [%d] Range [%f]", sonarId.c_str(), msg->header.seq, sonarRange);

  // For each sensor we are using keep a running weighted average
  if (sonarId == "sonar_3") {
      dist_front = (sonarRange * range_avg_factor) + (dist_front * ((double)(1.0) - range_avg_factor));
  }
  if (sonarId == "sonar_2") {
      dist_right = (sonarRange * range_avg_factor) + (dist_right * ((double)(1.0) - range_avg_factor));
  }
  if (sonarId == "sonar_1") {
      dist_left  = (sonarRange * range_avg_factor) + (dist_left * ((double)(1.0) - range_avg_factor));
  }

}


/**
 * Callback function executes when new cmd_vel speed commands come in
 * Here we simply keep last speed values seen
 */
void SonarDetector::cmdVelCallback(const ::geometry_msgs::Twist::ConstPtr& msg)
{
  // Pick off velocity and angular twist
  forwardVel  = (double)(1.0) * (float)(msg->linear.x);
  angularVel  = (double)(msg->angular.z);

  // Because cmd_vel is a rapidly repeating topic we don't always print each one
  ROS_DEBUG("Got Twist: Lin.x %6.2f  Ang.z %6.2f", forwardVel, angularVel);
}

SonarDetector::SonarDetector() {
    // NodeHandle is the main access point to communications with the ROS system.
    ros::NodeHandle nh;

    ROS_INFO("Sonar Detector Node Started");

    // Set the distances for detection of possible collision
    nh.param<double>("collision_thresh_near",  col_thresh_near, COL_THRESH_NEAR);
    nh.param<double>("collision_thresh_far",   col_thresh_far,  COL_THRESH_FAR);

    nh.param<double>("range_avg_factor",  range_avg_factor, RANGE_AVG_FACTOR);

    // Allow for selectively using a subset of sensors
    nh.param<int>("enabled_sensors", enabled_sensors, ENABLED_SENSORS);

    nh.param<int>("exit_collision_on_reverse", exit_on_rev_mode, 1);
    nh.param<double>("loop_rate", loop_rate, DETECTOR_LOOP_RATE);

    // we will listen to the combined topic that holds all sonar frames
    sonarSub = nh.subscribe("sonars", 100, &SonarDetector::sonarCallback, this);

    // We can use backup commands to exit the mode of being in a collision
    cmdVelSub = nh.subscribe("cmd_vel", 100, &SonarDetector::cmdVelCallback, this);

    // Create a ROS publisher to publish string commands to ubiquity_motor node
    sysControlPub = nh.advertise<std_msgs::String>(SYS_CONTROL_TOPIC, 1000);

    // Initialize distances for the sensors to outside of detection range
    dist_front = col_thresh_far * 2;
    dist_right = col_thresh_far * 2;
    dist_left  = col_thresh_far * 2;
    forwardVel = (double)(0.0);
    angularVel = (double)(0.0);

    // zero state values for collision and/or if motors are forced to be stopped
    zeroMotorSpeed = 0;
    collisionBits  = 0;

    ROS_INFO("Sonar Detector Node Ready");
}


/**
 * see if we detect onject(s) in front closer than distance threshold
 * This uses slightly longer distances for left and right detection
 * The thresholds are based on the Magni Sonar Board sonar placements and angles
 *
 * Return 0 if no object too close, return non-zero for objects within detection.
 *     Returns bit-encoded non-zero value for which sensors detect a collision
 */
int SonarDetector::collisionDetect(double threshold) {

    // Detection thresholds depend on physical placement of sensors
    double thresh_front = threshold;  // Front uses the input value

    // Right and left use input value adjusted for the 45 deg angle and location of sensor
    double thresh_right = threshold * (double)(1.4) - (double)(0.05);
    double thresh_left  = threshold * (double)(1.4) + (double)(0.10);

    int    retCode = 0;

    // Set appropriate collision bit for any sensors that see object too close
    if (((enabled_sensors & SENSOR_FRONT) != 0) && (dist_front <= thresh_front)) {
        retCode |= SENSOR_FRONT;
    }
    if (((enabled_sensors & SENSOR_LEFT) != 0) &&  (dist_left  <= thresh_left)) {
        retCode |= SENSOR_LEFT;
    }
    if (((enabled_sensors & SENSOR_RIGHT) != 0) && (dist_right <= thresh_right)) {
        retCode |= SENSOR_RIGHT;
    }

    return retCode;
}


/**
 * Returns 0 if no object in collision zone.
 * Non-zero indicates one or more sensors detect a colision
 *
 * Return 0 if no object too close, return non-zero for objects within detection.
 *     Returns bit-encoded non-zero value for which sensors detect a collision
 */
int SonarDetector::isCollision() {
    return collisionBits;
}


/**
 * The main loop of the node remains in this call 
 * Callbacks change state values that this logic here acts upon
 */
void SonarDetector::run() {
  int32_t loopIdx = 1;

  ros::Rate loopRate(loop_rate);       // This sets loops per second

  while (ros::ok()) {
      ros::spinOnce();
      loopIdx += 1;

      // Look for objects too close and thus in the collision detection range
      collisionBits = collisionDetect(col_thresh_near);

      std_msgs::String msg;
      std::stringstream ss;
      msg.data = ss.str();
      if (collisionBits != 0) {
          if ((exit_on_rev_mode != 0) && (forwardVel < (double)(0.0))) {
              // If going in reverse is allowed to exit collision mode we do that here
              if (zeroMotorSpeed != 0) {
                  ROS_INFO("Enable Motor Speed due to reverse speed: Left %f  Front %f  Right %f",
                      dist_left, dist_front, dist_right);
                  // Drop out of collision prevention and allow motors to be set to non-zero velocity
                  ss << "speed_control enable";
              }
              zeroMotorSpeed = 0;    // Get out of zero speed mode
          } else if (zeroMotorSpeed == 0) {
              if ((exit_on_rev_mode != 0) && (forwardVel < (double)(0.0))) {
                  // if reverse speed allows not going into stop mode then don't go there
                  zeroMotorSpeed = 0;
              } else {
                  zeroMotorSpeed = 1;
                  ROS_INFO("Disable Motor Speed: Left %f  Front %f  Right %f", dist_left, dist_front, dist_right);
                  // Drop into collision prevention and make motors be zero speed
                  ss << "speed_control disable";
              }
          } 
      } else {
          // No object within nearest threshold. Look for collision exit conditions
          if ((collisionDetect(col_thresh_far) == 0) || ((exit_on_rev_mode != 0) && (forwardVel < (double)(0.0)))) {
              collisionBits = 0;
              // To drop out of collision avoid we have a little hysteresys
              if (zeroMotorSpeed != 0) {
                  zeroMotorSpeed = 0;
                  ROS_INFO("Enable Motor Speed: Left %f  Front %f  Right %f", dist_left, dist_front, dist_right);
                  // Drop out of collision prevention and allow motors to be set to non-zero velocity
                  ss << "speed_control enable";
              }
          }
      }

      if ((loopIdx % ((int)loop_rate*2)) == 1) {
          ROS_INFO("Sonar Ranges: Left %f  Front %f  Right %f range [%f - %f] zeroMotors %d collisionBits %d",
              dist_left, dist_front, dist_right, col_thresh_near, col_thresh_far, zeroMotorSpeed, collisionBits);
      }

      if (ss.str().length() > 0) {
          msg.data = ss.str();
          sysControlPub.publish(msg);
      }
      
      loopRate.sleep();
  }
}

//  Entry point
int main(int argc, char ** argv) {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     */
    ros::init(argc, argv, "sonar_detector");
    SonarDetector sd_node;
    sd_node.run();

    return 0;
}
