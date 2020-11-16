#include "rotors_gazebo_plugins/gazebo_forcetorque_motor_plugin.h"
#include <chrono>

namespace gazebo {

GazeboForceTorqueMotor::~GazeboForceTorqueMotor() {
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void GazeboForceTorqueMotor::InitializeParams() {}

void GazeboForceTorqueMotor::Load(physics::ModelPtr _model,
                                  sdf::ElementPtr _sdf) {
  double noise_normal_torque;
  double noise_uniform_torque;
  std::string link_name;

  namespace_.clear();

  wrench_queue_.clear();

  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  else
    gzerr << "[gazebo_ft_motor] Please specify a robotNamespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

  if (_sdf->HasElement("linkName"))
    link_name = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_ft_motor] Please specify a linkName, where the "
             "force/torque is attached.\n";
  link_ = _model->GetLink(link_name);

  if (link_ == NULL)
    gzthrow("[gazebo_ft_motor] Couldn't find specified link \"" << link_name
                                                                << "\".");

  if (_sdf->HasElement("maxForce"))
    max_torque_ = _sdf->GetElement("maxForce")->Get<double>();
  else
    gzerr << "[gazebo_ft_motor] Please specify a maxForque.\n";

  if (_sdf->HasElement("maxTorque"))
    max_torque_ = _sdf->GetElement("maxTorque")->Get<double>();
  else
    gzerr << "[gazebo_ft_motor] Please specify a maxTorque.\n";

  if (_sdf->HasElement("randomEngineSeed")) {
    random_generator_.seed(
        _sdf->GetElement("randomEngineSeed")->Get<unsigned int>());
  } else {
    random_generator_.seed(
        std::chrono::system_clock::now().time_since_epoch().count());
  }

  getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_,
                           command_sub_topic_);
  getSdfParam<double>(_sdf, "noiseNormalTorque", noise_normal_torque, 0.0);
  getSdfParam<double>(_sdf, "noiseUniformTorque", noise_uniform_torque, 0.0);
  getSdfParam<int>(_sdf, "measurementDelay", measurement_delay_,
                   measurement_delay_);
  getSdfParam<int>(_sdf, "measurementDivisor", measurement_divisor_,
                   measurement_divisor_);
  getSdfParam<double>(_sdf, "unknownDelay", unknown_delay_, unknown_delay_);

  effort_n_ = NormalDistribution(0, noise_normal_torque);
  effort_u_ = UniformDistribution(-noise_uniform_torque, noise_uniform_torque);

  // Listen to the update event. This event is broadcast every simulation
  // iteration.
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboForceTorqueMotor::OnUpdate, this, _1));

  command_sub_ = node_handle_->subscribe(
      command_sub_topic_, 10, &GazeboForceTorqueMotor::CommandCallback, this);
}

// This gets called by the world update start event.
void GazeboForceTorqueMotor::OnUpdate(const common::UpdateInfo& _info) {
  if (!received_first_command_) {
    return;
  }

  if (gazebo_sequence_ % measurement_divisor_ == 0) {
    wrench_queue_.push_back(std::make_pair(
        gazebo_sequence_ + measurement_delay_, current_wrench_command_));
  }

  // Is it time to execute the control command?
  if (gazebo_sequence_ == wrench_queue_.front().first) {
    UpdateForces();
  }

  ++gazebo_sequence_;
}

void GazeboForceTorqueMotor::CommandCallback(
    const geometry_msgs::WrenchStampedPtr& msg) {
  current_wrench_command_ << msg->wrench.force.x, msg->wrench.force.y,
      msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
      msg->wrench.torque.z;
  received_first_command_ = true;
}

void GazeboForceTorqueMotor::UpdateForces() {
  Vector6d wrench_cmd = wrench_queue_.front().second;
  wrench_queue_.pop_front();

  for (int i = 0; i < 3; i++) {
    wrench_cmd[i] = limit(wrench_cmd[i], max_force_, -max_force_);
    wrench_cmd[i] +=
        effort_n_(random_generator_) + effort_u_(random_generator_);
    wrench_cmd[i + 3] = limit(wrench_cmd[i + 3], max_torque_, -max_torque_);
    wrench_cmd[i + 3] +=
        effort_n_(random_generator_) + effort_u_(random_generator_);
  }

  link_->AddRelativeForce(
      ignition::math::Vector3d(wrench_cmd[0], wrench_cmd[1], wrench_cmd[2]));
  link_->AddRelativeTorque(
      ignition::math::Vector3d(wrench_cmd[3], wrench_cmd[4], wrench_cmd[5]));
}

GZ_REGISTER_MODEL_PLUGIN(GazeboForceTorqueMotor);
}  // namespace gazebo
