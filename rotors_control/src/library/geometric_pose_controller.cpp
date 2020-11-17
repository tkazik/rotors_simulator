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

#include "rotors_control/geometric_pose_controller.h"

namespace rotors_control {

GeometricPoseController::GeometricPoseController()
    : initialized_params_(false), controller_active_(false) {
  InitializeParameters();
}

GeometricPoseController::~GeometricPoseController() {}

void GeometricPoseController::InitializeParameters() {
  // To make the tuning independent of the mass and inertia we divide here.
  normalized_position_gain_ =
      1.0 / vehicle_parameters_.mass_ * controller_parameters_.position_gain_;

  normalized_velocity_gain_ =
      1.0 / vehicle_parameters_.mass_ * controller_parameters_.velocity_gain_;

  normalized_attitude_gain_ =
      controller_parameters_.attitude_gain_.transpose() *
      vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ =
      controller_parameters_.angular_rate_gain_.transpose() *
      vehicle_parameters_.inertia_.inverse();

  initialized_params_ = true;
}

void GeometricPoseController::CalculateDesiredWrench(
    Eigen::Vector3d* force_B, Eigen::Vector3d* torque_B) const {
  assert(force_B);
  assert(torque_B);
  assert(initialized_params_);

  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    force_B->setZero();
    torque_B->setZero();
    return;
  }

  Eigen::Vector3d acceleration_B;
  ComputeDesiredAcceleration(&acceleration_B);

  Eigen::Vector3d angular_acceleration_B;
  ComputeDesiredAngularAcc(acceleration_B, &angular_acceleration_B);

  *force_B = vehicle_parameters_.mass_ * acceleration_B;
  *torque_B = vehicle_parameters_.inertia_ * angular_acceleration_B;
}

void GeometricPoseController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void GeometricPoseController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
  controller_active_ = true;
}

void GeometricPoseController::ComputeDesiredAcceleration(
    Eigen::Vector3d* acceleration_B) const {
  assert(acceleration_B);

  const Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

  Eigen::Vector3d position_error_B;
  position_error_B =
      R.transpose() * (odometry_.position - command_trajectory_.position_W);

  Eigen::Vector3d velocity_error_B;
  velocity_error_B =
      R.transpose() * (odometry_.velocity - command_trajectory_.velocity_W);

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

  *acceleration_B =
      -1 * position_error_B.cwiseProduct(normalized_position_gain_) -
      velocity_error_B.cwiseProduct(normalized_velocity_gain_) -
      odometry_.angular_velocity.cross(odometry_.velocity) +
      R.transpose() * vehicle_parameters_.gravity_ * e_3 +
      R.transpose() * command_trajectory_.acceleration_W;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on
// SE(3) -> Simplified for 6DOF geometric control.
void GeometricPoseController::ComputeDesiredAngularAcc(
    const Eigen::Vector3d& acceleration,
    Eigen::Vector3d* angular_acceleration_B) const {
  assert(angular_acceleration_B);

  Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
  Eigen::Matrix3d R_des =
      command_trajectory_.orientation_W_B.toRotationMatrix();

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix =
      0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // Angular velocity command is in body frame.
  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity -
  R_des.transpose() * R * command_trajectory_.angular_velocity_W;

  // TODO (@kajabo) double add feed-forward COM offset

  *angular_acceleration_B =
      -1 * angle_error.cwiseProduct(normalized_attitude_gain_) -
      angular_rate_error.cwiseProduct(normalized_angular_rate_gain_) +
      vehicle_parameters_.inertia_.inverse() *
          odometry_.angular_velocity.cross(vehicle_parameters_.inertia_ *
                                           odometry_.angular_velocity);
}
}  // namespace rotors_control