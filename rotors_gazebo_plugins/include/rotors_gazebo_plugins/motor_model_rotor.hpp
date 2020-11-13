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

#ifndef ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_ROTOR_H
#define ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_ROTOR_H

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

namespace gazebo {

class MotorModelRotor : public MotorModel {
 public:
  MotorModelRotor(const sdf::ElementPtr _motor, const physics::JointPtr _joint,
                   const physics::LinkPtr _link)
      : MotorModel(),
        spin_direction_(spin::CCW),
        time_constant_up_(kDefaultTimeConstantUp),
        time_constant_down_(kDefaultTimeConstantDown),
        max_rot_velocity_(kDefaultMaxRotVelocity),
        min_rot_velocity_(kDefaultMinRotVelocity),
        motor_constant_(kDefaultMotorConstant),
        moment_constant_(kDefaultMomentConstant),
        rotor_drag_coefficient_(kDefaultRotorDragCoefficient),
        rolling_moment_coefficient_(kDefaultRollingMomentCoefficient),
        rotor_velocity_slowdown_sim_(kDefaultRotorVelocitySlowdownSim) {
    motor_ = _motor;
    joint_ = _joint;
    link_ = _link;
    InitializeParams();
  }

  virtual ~MotorModelRotor() {}

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
  std::string joint_name_;
  std::string link_name_;
  int spin_direction_;
  double time_constant_up_;
  double time_constant_down_;
  double max_rot_velocity_;
  double min_rot_velocity_;
  double motor_constant_;
  double moment_constant_;
  double rotor_drag_coefficient_;
  double rolling_moment_coefficient_;
  double rotor_velocity_slowdown_sim_;

  sdf::ElementPtr motor_;
  physics::JointPtr joint_;
  physics::LinkPtr link_;

  void InitializeParams() {

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
    getSdfParam<double>(motor_, "rotorDragCoefficient", rotor_drag_coefficient_,
                        rotor_drag_coefficient_);
    getSdfParam<double>(motor_, "rollingMomentCoefficient",
                        rolling_moment_coefficient_,
                        rolling_moment_coefficient_);
    getSdfParam<double>(motor_, "maxRotVelocity", max_rot_velocity_,
                        max_rot_velocity_);
    getSdfParam<double>(motor_, "minRotVelocity", min_rot_velocity_,
                        min_rot_velocity_);
    getSdfParam<double>(motor_, "motorConstant", motor_constant_,
                        motor_constant_);
    getSdfParam<double>(motor_, "momentConstant", moment_constant_,
                        moment_constant_);
    getSdfParam<double>(motor_, "timeConstantUp", time_constant_up_,
                        time_constant_up_);
    getSdfParam<double>(motor_, "timeConstantDown", time_constant_down_,
                        time_constant_down_);
    getSdfParam<double>(motor_, "rotorVelocitySlowdownSim",
                        rotor_velocity_slowdown_sim_, 10);
  }

  void Publish(){} // No publishing here

  void UpdateForcesAndMoments(){
    // TODO: add force/moment updates 
  }
};

}  // namespace gazebo

#endif  // ROTORS_GAZEBO_PLUGINS_MOTOR_MODEL_ROTOR_H
