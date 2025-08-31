/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Acorn Pooley, Michael Lautman */

// This code goes with the interactivity tutorial

#include "interactivity/interactive_robot.h"
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <chrono>
#include <moveit_msgs/msg/detail/display_robot_state__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

// default world object position is just in front and left of Panda robot.
const Eigen::Isometry3d
    InteractiveRobot::DEFAULT_WORLD_OBJECT_POSE_(Eigen::Isometry3d(Eigen::Translation3d(0.25, -0.5, 0.5)));

// size of the world geometry cube
const double InteractiveRobot::WORLD_BOX_SIZE_ = 0.15;

// minimum delay between calls to callback function
const rclcpp::Duration InteractiveRobot::min_delay_(std::chrono::milliseconds(100));

const auto logger = rclcpp::get_logger("interactive_robot");

InteractiveRobot::InteractiveRobot(const std::string& robot_description, const std::string& robot_topic,
                                   const std::string& marker_topic, const std::string& imarker_topic)
  : user_data_(nullptr)
  , nh_(rclcpp::Node::make_shared("interactive_robot"))  // this node handle is used to create the publishers
  // create publishers for markers and robot state
  , robot_state_publisher_(nh_->create_publisher<moveit_msgs::msg::DisplayRobotState>(robot_topic, rclcpp::QoS(10)))
  , world_state_publisher_(nh_->create_publisher<visualization_msgs::msg::Marker>(marker_topic, rclcpp::QoS(10)))
  // create an interactive marker server for displaying interactive markers
  , interactive_marker_server_(imarker_topic, nh_)
  , imarker_robot_(nullptr)
  , imarker_world_(nullptr)
  // load the robot description
  , rm_loader_(nh_, robot_description)
  , group_(nullptr)
  , average_callback_duration_(min_delay_)
{
  // get the RobotModel loaded from urdf and srdf files
  robot_model_ = rm_loader_.getModel();
  if (!robot_model_)
  {
    RCLCPP_ERROR(logger, "Could not load robot description");
    throw RobotLoadException();
  }

  // create a RobotState to keep track of the current robot pose
  robot_state_.reset(new moveit::core::RobotState(robot_model_));
  if (!robot_state_)
  {
    RCLCPP_ERROR(logger, "Could not get RobotState from Model");
    throw RobotLoadException();
  }
  robot_state_->setToDefaultValues();

  // Prepare to move the "panda_arm" group
  group_ = robot_state_->getJointModelGroup("panda_arm");
  std::string end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);

  // Create a marker to control the "panda_arm" group
  imarker_robot_ =
      std::make_unique<IMarker>(interactive_marker_server_, "robot", desired_group_end_link_pose_, "/panda_link0",
                                std::bind(&InteractiveRobot::movedRobotMarkerCallback, this, std::placeholders::_1),
                                IMarker::BOTH);

  // create an interactive marker to control the world geometry (the yellow cube)
  desired_world_object_pose_ = DEFAULT_WORLD_OBJECT_POSE_;
  imarker_world_ =
      std::make_unique<IMarker>(interactive_marker_server_, "world", desired_world_object_pose_, "/panda_link0",
                                std::bind(&InteractiveRobot::movedWorldMarkerCallback, this, std::placeholders::_1),
                                IMarker::POS);

  // start publishing timer.
  publish_timer_ = nh_->create_wall_timer(average_callback_duration_.to_chrono<std::chrono::milliseconds>(),
                                          std::bind(&InteractiveRobot::updateAll, this));

  schedule_request_count_ = 0;
  init_time_ = nh_->now();
  last_callback_time_ = init_time_;
  RCLCPP_INFO(logger, "InteractiveRobot ready");
  updateAll();
}

InteractiveRobot::~InteractiveRobot()
{
}

// callback called when marker moves.  Moves right hand to new marker pose.
void InteractiveRobot::movedRobotMarkerCallback(
    InteractiveRobot* robot, const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  Eigen::Isometry3d pose;
  tf2::fromMsg(feedback->pose, pose);
  robot->setGroupPose(pose);
}

// callback called when marker moves.  Moves world object to new pose.
void InteractiveRobot::movedWorldMarkerCallback(
    InteractiveRobot* robot, const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  Eigen::Isometry3d pose;
  tf2::fromMsg(feedback->pose, pose);
  robot->setWorldObjectPose(pose);
}

/* Calculate new positions and publish results to rviz */
void InteractiveRobot::updateAll()
{
  publishWorldState();

  if (robot_state_->setFromIK(group_, desired_group_end_link_pose_, 0.1))
  {
    publishRobotState();
  }
}

// change which group is being manipulated
void InteractiveRobot::setGroup(const std::string& name)
{
  const moveit::core::JointModelGroup* group = robot_state_->getJointModelGroup(name);
  if (!group)
  {
    RCLCPP_ERROR_STREAM(logger, "No joint group named " << name);
    if (!group_)
      throw RobotLoadException();
  }
  group_ = group;
  std::string end_link = group_->getLinkModelNames().back();
  desired_group_end_link_pose_ = robot_state_->getGlobalLinkTransform(end_link);
  if (imarker_robot_)
  {
    imarker_robot_->move(desired_group_end_link_pose_);
  }
}

// return current group name
const std::string& InteractiveRobot::getGroupName() const
{
  return group_->getName();
}

/* remember new desired robot pose and schedule an update */
void InteractiveRobot::setGroupPose(const Eigen::Isometry3d& pose)
{
  desired_group_end_link_pose_ = pose;
}

/* publish robot pose to rviz */
void InteractiveRobot::publishRobotState()
{
  moveit_msgs::msg::DisplayRobotState msg;
  moveit::core::robotStateToRobotStateMsg(*robot_state_, msg.state);
  robot_state_publisher_->publish(msg);
}

/* remember new world object position and schedule an update */
void InteractiveRobot::setWorldObjectPose(const Eigen::Isometry3d& pose)
{
  desired_world_object_pose_ = pose;
}

/* publish world object position to rviz */
void InteractiveRobot::publishWorldState()
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "/panda_link0";
  marker.header.stamp = nh_->now();
  marker.ns = "world_box";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = WORLD_BOX_SIZE_;
  marker.scale.y = WORLD_BOX_SIZE_;
  marker.scale.z = WORLD_BOX_SIZE_;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.4f;
  marker.lifetime = rclcpp::Duration(0, 500000000);  // 0.5 seconds
  marker.pose = tf2::toMsg(desired_world_object_pose_);
  world_state_publisher_->publish(marker);
}

/* get world object pose and size */
void InteractiveRobot::getWorldGeometry(Eigen::Isometry3d& pose, double& size)
{
  pose = desired_world_object_pose_;
  size = WORLD_BOX_SIZE_;
}
