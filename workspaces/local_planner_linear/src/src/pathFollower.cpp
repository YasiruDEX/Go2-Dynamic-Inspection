#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include <geometry_msgs/msg/twist.hpp>  // Change from twist_stamped.hpp

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
int safetyStop = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;

// Goal pose tracking for rotation at goal
bool goalPoseReceived = false;
float goalPoseYaw = 0;       // desired yaw from goal pose (world frame)
bool atGoalRotating = false;  // state: we've arrived and are now rotating
bool goalRotationDone = false; // latch: once we reach the target angle, stop permanently

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
    path.poses[i].pose.orientation = pathIn->poses[i].pose.orientation;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;

  // When a new path with >1 points arrives, we're no longer in "at goal rotating" mode
  if (pathSize > 1) {
    atGoalRotating = false;
    goalRotationDone = false;
  }
}

// Separate subscriber for the goal pose (with orientation)
void goalPoseHandler(const geometry_msgs::msg::PoseStamped::ConstSharedPtr goalPose)
{
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = goalPose->pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);
  goalPoseYaw = yaw;
  goalPoseReceived = true;
  atGoalRotating = false;  // reset rotation state on new goal
  goalRotationDone = false; // reset done latch on new goal
  RCLCPP_INFO(nh->get_logger(), "Goal pose received: yaw = %.2f deg", yaw * 180.0 / PI);
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  if (joySpeedRaw > 0.05) joyTime = nh->now().seconds();
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  // if (joy->axes[2] > -0.1) {
  //   autonomyMode = false;
  // } else {
  //   autonomyMode = true;
  // }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/lidar_odometry/pose", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  // Subscribe to goal pose for orientation target
  auto subGoalPose = nh->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 5, goalPoseHandler);

  // auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);  // Change from TwistStamped

  geometry_msgs::msg::Twist cmd_vel;  // Change from TwistStamped


  // geometry_msgs::msg::TwistStamped cmd_vel;
  // cmd_vel.header.frame_id = "base_link";

  if (autonomyMode) {
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (pathInit) {

      // ===== GOAL ROTATION MODE =====
      // When localPlanner publishes pathSize<=1, the robot has arrived at the goal.
      // If we have a goal orientation and noRotAtGoal is false, rotate in place.
      int pathSize = path.poses.size();

      // Compute vehicle position relative to when the path was received
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      bool atGoalPosition = (pathSize <= 1) || (endDis < stopDisThre);

      // Log the decision state every 200ms
      RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 200,
        "[STATE] pathSize=%d, endDis=%.2f, atGoal=%d, goalPoseReceived=%d, noRotAtGoal=%d, atGoalRotating=%d, goalRotationDone=%d, vehicleYaw=%.2f deg, goalPoseYaw=%.2f deg",
        pathSize, endDis, atGoalPosition, goalPoseReceived, noRotAtGoal, atGoalRotating, goalRotationDone,
        vehicleYaw * 180.0 / PI, goalPoseYaw * 180.0 / PI);

      if (atGoalPosition && goalPoseReceived && !noRotAtGoal && !goalRotationDone) {
        // We are at the goal position — enter/stay in rotation mode
        atGoalRotating = true;
        vehicleSpeed = 0;  // no linear motion

        if (goalRotationDone) {
          // Already reached target angle — stay stopped
          vehicleYawRate = 0;
          RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000,
            "[GOAL_ROT] DONE (latched). vehicleYaw=%.2f deg, goalYaw=%.2f deg",
            vehicleYaw * 180.0 / PI, goalPoseYaw * 180.0 / PI);
        } else {
          // Compute yaw error in world frame
          float yawError = goalPoseYaw - vehicleYaw;
          if (yawError > PI) yawError -= 2 * PI;
          else if (yawError < -PI) yawError += 2 * PI;

          RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 200,
            "[GOAL_ROT] yawError=%.4f rad (%.2f deg), dirDiffThre=%.4f rad (%.2f deg), within=%d",
            yawError, yawError * 180.0 / PI, dirDiffThre, dirDiffThre * 180.0 / PI, (fabs(yawError) < dirDiffThre));

          if (fabs(yawError) < dirDiffThre) {
            // Within threshold — latch done and stop permanently
            vehicleYawRate = 0;
            goalRotationDone = true;
            RCLCPP_INFO(nh->get_logger(),
              "[GOAL_ROT] === COMPLETE === yawError=%.2f deg (thre=%.2f deg). LATCHING DONE.", 
              yawError * 180.0 / PI, dirDiffThre * 180.0 / PI);
          } else {
            // Proportional yaw control with reduced max rate to prevent overshoot
            float goalMaxYawRate = maxYawRate * 0.5 * PI / 180.0;  // half of maxYawRate
            vehicleYawRate = -stopYawRateGain * yawError;
            float rawYawRate = vehicleYawRate;
            if (vehicleYawRate > goalMaxYawRate) vehicleYawRate = goalMaxYawRate;
            else if (vehicleYawRate < -goalMaxYawRate) vehicleYawRate = -goalMaxYawRate;

            RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 200,
              "[GOAL_ROT] ROTATING: yawErr=%.2f deg, rawRate=%.4f, clampedRate=%.4f, maxRate=%.4f, gain=%.2f, cmd_vel.z=%.4f", 
              yawError * 180.0 / PI, rawYawRate, vehicleYawRate, goalMaxYawRate, stopYawRateGain, vehicleYawRate);
          }
        }

        if (safetyStop >= 1) vehicleSpeed = 0;
        if (safetyStop >= 2) vehicleYawRate = 0;

        pubSkipCount--;
        if (pubSkipCount < 0) {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = vehicleYawRate;
          pubSpeed->publish(cmd_vel);
          pubSkipCount = pubSkipNum;
        }

      } else if (atGoalPosition) {
        // ===== AT GOAL, NO ROTATION NEEDED =====
        vehicleSpeed = 0;
        vehicleYawRate = 0;
        atGoalRotating = false;

        if (safetyStop >= 1) vehicleSpeed = 0;
        if (safetyStop >= 2) vehicleYawRate = 0;

        pubSkipCount--;
        if (pubSkipCount < 0) {
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          pubSpeed->publish(cmd_vel);
          pubSkipCount = pubSkipNum;
        }

        RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000,
          "[AT_GOAL] Stopped. endDis=%.2f, goalPoseRcvd=%d, rotDone=%d", 
          endDis, goalPoseReceived, goalRotationDone);

      } else {
        // ===== NORMAL PATH FOLLOWING MODE =====
        atGoalRotating = false;

        float disX, disY, dis;
        while (pathPointID < pathSize - 1) {
          disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
          disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
          dis = sqrt(disX * disX + disY * disY);
          if (dis < lookAheadDis) {
            pathPointID++;
          } else {
            break;
          }
        }

        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        float pathDir = atan2(disY, disX);

        float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        else if (dirDiff < -PI) dirDiff += 2 * PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        else if (dirDiff < -PI) dirDiff += 2 * PI;

        if (twoWayDrive) {
          double time = nh->now().seconds();
          if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
            navFwd = false;
            switchTime = time;
          } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
            navFwd = true;
            switchTime = time;
          }
        }

        if (autonomyMode && nh->now().seconds() - joyTime > joyToSpeedDelay && joySpeed == 0) {
          joySpeed = autonomySpeed / maxSpeed;
          if (joySpeed < 0) joySpeed = 0;
          else if (joySpeed > 1.0) joySpeed = 1.0;
        }

        float joySpeed2 = maxSpeed * joySpeed;
        if (!navFwd) {
          dirDiff += PI;
          if (dirDiff > PI) dirDiff -= 2 * PI;
          joySpeed2 *= -1;
        }

        if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
        else vehicleYawRate = -yawRateGain * dirDiff;

        if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
        else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;

        if (joySpeed2 == 0 && !autonomyMode) {
          vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
        } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
          vehicleYawRate = 0;
        }

        if (pathSize <= 1) {
          joySpeed2 = 0;
        } else if (endDis / slowDwnDisThre < joySpeed) {
          joySpeed2 *= endDis / slowDwnDisThre;
        }

        float joySpeed3 = joySpeed2;
        if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0) joySpeed3 *= slowRate1;
        else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) joySpeed3 *= slowRate2;

        if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
          if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
          else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
        } else {
          if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
          else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
        }

        if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
          vehicleSpeed = 0;
          vehicleYawRate = 0;
        }

        if (safetyStop >= 1) vehicleSpeed = 0;
        if (safetyStop >= 2) vehicleYawRate = 0;

        pubSkipCount--;
        if (pubSkipCount < 0) {
          if (fabs(vehicleSpeed) <= maxAccel / 100.0) cmd_vel.linear.x = 0; 
          else cmd_vel.linear.x = vehicleSpeed;  
          cmd_vel.angular.z = vehicleYawRate;  
          pubSpeed->publish(cmd_vel);

          RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000, "Vel: %.2f, Yaw: %.2f, JoyS: %.2f, Auto: %d, Safe: %d, Path: %d", 
                      vehicleSpeed, vehicleYawRate, joySpeed, autonomyMode, safetyStop, pathInit);

          RCLCPP_INFO_THROTTLE(nh->get_logger(), *nh->get_clock(), 1000, "DEBUG: pathSize=%d, dirDiff=%.2f (thre=%.2f), dis=%.2f (thre=%.2f), joySpeed=%.2f, joySpeed2=%.2f, joySpeed3=%.2f, vehicleSpeed=%.2f, stopTime=%.2f, slowTime1=%.2f, odomTime=%.2f, stopInitTime=%.2f", 
                      pathSize, dirDiff, dirDiffThre, dis, stopDisThre, joySpeed, joySpeed2, joySpeed3, vehicleSpeed, stopTime, slowTime1, odomTime, stopInitTime);

          pubSkipCount = pubSkipNum;
        }
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
