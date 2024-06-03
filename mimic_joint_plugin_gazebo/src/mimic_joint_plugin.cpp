#include "mimic_joint_plugin_gazebo/mimic_joint_plugin.hpp"

namespace gazebo {

MimicJointPlugin::MimicJointPlugin() {}

MimicJointPlugin::~MimicJointPlugin() {}

// Inherited
void MimicJointPlugin::Load(gazebo::physics::ModelPtr _model,
                            sdf::ElementPtr _sdf) {
  model_ = _model;

  if (!model_) {
    std::cerr << "\nParent model is NULL! MimicJointPlugin could not be loaded."
              << std::endl;
    return;
  }
  world_ = model_->GetWorld();

  debug = _sdf->Get<bool>("debug", false).first;

  if (debug) {
    auto joints = model_->GetJoints();
    std::cerr << "\nName of the joints present in the model:" << std::endl;
    for (const auto &joint : joints) {
      std::cerr << joint->GetName() << std::endl;
    }
  }

  joint_name_ = _sdf->Get<std::string>("joint", "joint").first;
  if (!model_->GetJoint(joint_name_)) {
    std::cerr << "\nJoint element " << joint_name_
              << " not present in sdf. MimicJointPlugin could not be loaded"
              << std::endl;
    return;
  }
  joint_ = model_->GetJoint(joint_name_);

  mimic_joint_name_ =
      _sdf->Get<std::string>("mimic_joint", "mimic_joint").first;
  if (!model_->GetJoint(mimic_joint_name_)) {
    std::cerr << "\nMimic joint element " << mimic_joint_name_
              << " not present in sdf. MimicJointPlugin could not be loaded"
              << std::endl;
    return;
  }
  mimic_joint_ = model_->GetJoint(mimic_joint_name_);

  multiplier_ = _sdf->Get<double>("multiplier", 1.0).first;
  offset_ = _sdf->Get<double>("offset", 0.0).first;

  update_period_ = _sdf->Get<double>("update_period", 0.1).first;

  use_pid = _sdf->Get<bool>("use_pid", false).first;
  kp = _sdf->Get<double>("kp", 100.0).first;
  ki = _sdf->Get<double>("ki", 100.0).first;
  kd = _sdf->Get<double>("kd", 100.0).first;

  jointController.reset(new physics::JointController(model_));
  jointController->AddJoint(mimic_joint_);
  mimic_joint_scoped_name = mimic_joint_->GetScopedName();
  jointController->SetVelocityPID(mimic_joint_scoped_name,
                                  common::PID(kp, ki, kd));

  std::cout << "MimicJointPlugin starting connection." << std::endl;

  // Listen to the update event (broadcast every simulation iteration)
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MimicJointPlugin::OnUpdate, this, std::placeholders::_1));

  std::cout << "MimicJointPlugin finished loading." << std::endl;
}

// Inherited
void MimicJointPlugin::Reset() {}

void MimicJointPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {
  std::lock_guard<std::mutex> scoped_lock(lock_);

  double seconds_since_last_update =
      (_info.simTime - last_update_time_).Double();

  if (seconds_since_last_update < update_period_) {
    return;
  }

  auto mimic_angle = joint_->Position(0) * multiplier_ + offset_;

  if (debug) {
    std::cerr << "target mimic_angle: " << mimic_angle << std::endl;
  }

  if (use_pid) {
    jointController->SetPositionTarget(mimic_joint_scoped_name, mimic_angle);
  } else {
    mimic_joint_->SetPosition(0, mimic_angle,
                              true); // (axis,position, preserve world velocity)
  }

  last_update_time_ = _info.simTime;
}

} // namespace gazebo
