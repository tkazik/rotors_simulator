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

#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_SERVO_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_SERVO_H

// SYSTEM
// #include <stdio.h>

// 3RD PARTY
// #include <boost/bind.hpp>
#include <Eigen/Core>
// #include <gazebo/common/common.hh>
// #include <gazebo/common/Plugin.hh>
// #include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// USER
#include "rotors_gazebo_plugins/common.h"
#include "rotors_gazebo_plugins/motor_model.hpp"

enum class ControlMode { kVelocity, kPosition, kForce };

namespace gazebo {

class MotorModelServo : public MotorModel {
 public:
  MotorModelServo(const sdf::ElementPtr _motor, const physics::JointPtr _joint,
                   const physics::LinkPtr _link)
      : MotorModel(),
        type_(ControlMode::kPosition),
        spin_direction_(spin::CCW),
        max_rot_velocity_(kDefaultMaxRotVelocity),
        max_torque_(kDefaultMaxTorque),
        max_rot_position_(kDefaultMaxRotPosition),
        min_rot_position_(kDefaultMinRotPosition),
        position_zero_offset_(kDefaultPositionOffset) {
    motor_ = _motor;
    joint_ = _joint;
    link_ = _link;
    InitializeParams();
  }

  virtual ~MotorModelServo() {}

  void GetActuatorState(/*&position, &velocity, &effort*/){
    // Returns actuator position, velocity and effort
    UpdateForcesAndMoments();
    // TODO: return results.
  }

  void SetActuatorReference(/*position, velocity, effort*/){
    // Set reference actuator position, velocity and effort (type dependent)
    // TODO: Set refs.
  }

 protected:
  // Parameters
  ControlMode type_;
  std::string joint_name_;
  std::string link_name_;
  int spin_direction_;
  double max_rot_velocity_;
  double max_torque_;
  double max_rot_position_;
  double min_rot_position_;
  double position_zero_offset_;
  double p_gain_;
  double i_gain_;
  double d_gain_;

  sdf::ElementPtr motor_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;

  void InitializeParams() {
    // Check motor type.
    if (motor_->HasElement("controlMode")) {
      std::string motor_type =
          motor_->GetElement("controlMode")->Get<std::string>();
      if (motor_type == "velocity")
        type_ = ControlMode::kVelocity;
      else if (motor_type == "position")
        type_ = ControlMode::kPosition;
      else if (motor_type == "force") {
        type_ = ControlMode::kForce;
      } else
        gzwarn << "[single_motor] controlMode not valid, using position.\n";
    } else {
      gzwarn << "[single_motor] controlMode not specified, using position.\n";
    }

    // Check spin direction.
    if (motor_->HasElement("spinDirection")) {
      std::string spin_direction =
          motor_->GetElement("spinDirection")->Get<std::string>();
      if (spin_direction == "cw")
        spin_direction_ = spin::CW;
      else if (spin_direction == "ccw")
        spin_direction = spin::CCW;
      else
        gzerr << "[single_motor] Spin not valid, using 'ccw.'\n";
    } else {
      gzwarn << "[single_motor] spinDirection not specified, using ccw.\n";
    }
    getSdfParam<double>(motor_, "maxRotVelocity", max_rot_velocity_,
                        max_rot_velocity_);
    getSdfParam<double>(motor_, "maxRotPosition", max_rot_position_,
                        max_rot_position_);
    getSdfParam<double>(motor_, "maxRotPosition", min_rot_position_,
                        min_rot_position_);
    getSdfParam<double>(motor_, "zeroOffset", position_zero_offset_,
                        position_zero_offset_);
  }

  void Publish(){} // No publishing here

  void UpdateForcesAndMoments(){
    // TODO: add force/moment updates 
  }
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_SERVO_H
