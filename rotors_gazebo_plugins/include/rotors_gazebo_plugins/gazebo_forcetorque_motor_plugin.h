#ifndef ROTORS_GAZEBO_PLUGINS_FORCE_TORQUE_MOTOR_H
#define ROTORS_GAZEBO_PLUGINS_FORCE_TORQUE_MOTOR_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include "rotors_gazebo_plugins/common.h"


namespace gazebo {
// Default values
static const std::string kDefaultCommandSubTopic = "command/wrench";
static const double kDefaultMaxForce = 1e+16;
static const double kDefaultMaxTorque = 1e+16;
static const double kDefaultMaxAngleErrorIntegral = 1.0;
static constexpr int kDefaultMeasurementDelay = 0;
static constexpr int kDefaultMeasurementDivisor = 1;
static constexpr int kDefaultGazeboSequence = 0;
static constexpr double kDefaultUnknownDelay = 0.0;


class GazeboForceTorqueMotor : public ModelPlugin
{
 public:
  typedef std::normal_distribution<> NormalDistribution;
  typedef std::uniform_real_distribution<> UniformDistribution;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef std::deque<std::pair<int, Vector6d> > WrenchCommandQueue;

  GazeboForceTorqueMotor()
      : ModelPlugin(),
        random_generator_(random_device_()),
        command_sub_topic_(kDefaultCommandSubTopic),
        max_force_(kDefaultMaxForce),
        max_torque_(kDefaultMaxTorque),
        measurement_delay_(kDefaultMeasurementDelay),
        measurement_divisor_(kDefaultMeasurementDivisor),
        unknown_delay_(kDefaultUnknownDelay),
        gazebo_sequence_(kDefaultGazeboSequence),
        received_first_command_(false),
        node_handle_(NULL) {
          current_wrench_command_.setZero();
        }

  virtual ~GazeboForceTorqueMotor();

  virtual void InitializeParams();

 protected:
  virtual void UpdateForces();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_sub_topic_;
  std::string namespace_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber command_sub_;

  std::random_device random_device_;
  std::mt19937 random_generator_;

  // motor parameters
  double max_force_;
  double max_torque_;

  // measurements parameters
  int measurement_delay_;
  int measurement_divisor_;
  int gazebo_sequence_;
  int motor_sequence_;
  double unknown_delay_;

  NormalDistribution effort_n_;
  UniformDistribution effort_u_;

  WrenchCommandQueue wrench_queue_;
  Vector6d current_wrench_command_;

  bool received_first_command_;

  physics::LinkPtr link_;
  /// \brief Pointer to the update event connection.
  event::ConnectionPtr updateConnection_;

  boost::thread callback_queue_thread_;
  void QueueThread();
  void CommandCallback(const geometry_msgs::WrenchStampedPtr& msg);
};
}

#endif // ROTORS_GAZEBO_PLUGINS_FORCE_TORQUE_MOTOR_H
