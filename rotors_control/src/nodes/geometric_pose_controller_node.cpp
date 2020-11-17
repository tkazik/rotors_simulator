/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "geometric_pose_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

GeometricPoseControllerNode::GeometricPoseControllerNode(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  InitializeParams();

  cmd_pose_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &GeometricPoseControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh_.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &GeometricPoseControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &GeometricPoseControllerNode::OdometryCallback, this);

  command_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(
     "command/wrench", 1);

  command_timer_ = nh_.createTimer(ros::Duration(0), &GeometricPoseControllerNode::TimedCommandCallback, this,
                                  true, false);
}

GeometricPoseControllerNode::~GeometricPoseControllerNode() { }

void GeometricPoseControllerNode::InitializeParams() {

  // Read parameters from rosparam.
  GetRosParameter(private_nh_, "position_gain/x",
                  geometric_pose_controller_.controller_parameters_.position_gain_.x(),
                  &geometric_pose_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(private_nh_, "position_gain/y",
                  geometric_pose_controller_.controller_parameters_.position_gain_.y(),
                  &geometric_pose_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(private_nh_, "position_gain/z",
                  geometric_pose_controller_.controller_parameters_.position_gain_.z(),
                  &geometric_pose_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(private_nh_, "velocity_gain/x",
                  geometric_pose_controller_.controller_parameters_.velocity_gain_.x(),
                  &geometric_pose_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(private_nh_, "velocity_gain/y",
                  geometric_pose_controller_.controller_parameters_.velocity_gain_.y(),
                  &geometric_pose_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(private_nh_, "velocity_gain/z",
                  geometric_pose_controller_.controller_parameters_.velocity_gain_.z(),
                  &geometric_pose_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(private_nh_, "attitude_gain/x",
                  geometric_pose_controller_.controller_parameters_.attitude_gain_.x(),
                  &geometric_pose_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(private_nh_, "attitude_gain/y",
                  geometric_pose_controller_.controller_parameters_.attitude_gain_.y(),
                  &geometric_pose_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(private_nh_, "attitude_gain/z",
                  geometric_pose_controller_.controller_parameters_.attitude_gain_.z(),
                  &geometric_pose_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(private_nh_, "angular_rate_gain/x",
                  geometric_pose_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &geometric_pose_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(private_nh_, "angular_rate_gain/y",
                  geometric_pose_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &geometric_pose_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(private_nh_, "angular_rate_gain/z",
                  geometric_pose_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &geometric_pose_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(private_nh_, &geometric_pose_controller_.vehicle_parameters_);
  geometric_pose_controller_.InitializeParameters();
}
void GeometricPoseControllerNode::Publish() {
}

void GeometricPoseControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  geometric_pose_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void GeometricPoseControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  geometric_pose_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void GeometricPoseControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  geometric_pose_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void GeometricPoseControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("GeometricPoseController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  geometric_pose_controller_.SetOdometry(odometry);

  Eigen::Vector3d cmd_force_B, cmd_torque_B;
  geometric_pose_controller_.CalculateDesiredWrench(&cmd_force_B, &cmd_torque_B);

  geometry_msgs::WrenchStampedPtr wrench_msg(new geometry_msgs::WrenchStamped);

  wrench_msg->header.stamp = odometry_msg->header.stamp;
  mav_msgs::vectorEigenToMsg(cmd_force_B, &wrench_msg->wrench.force);
  mav_msgs::vectorEigenToMsg(cmd_torque_B, &wrench_msg->wrench.torque);

  command_wrench_pub_.publish(wrench_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "geometric_pose_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::GeometricPoseControllerNode geometric_pose_controller_node(nh, private_nh);

  ros::spin();

  return 0;
}
