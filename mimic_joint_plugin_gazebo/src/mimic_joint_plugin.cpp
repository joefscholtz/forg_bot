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

  // Listen to the update event (broadcast every simulation iteration)
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MimicJointPlugin::OnUpdate, this, std::placeholders::_1));
}

// Inherited
void MimicJointPlugin::Reset() {}

void MimicJointPlugin::OnUpdate(const gazebo::common::UpdateInfo &_info) {
  auto mimic_angle = joint_->Position(0) * multiplier_ + offset_;
  mimic_joint_->SetPosition(0, mimic_angle, true);
}

} // namespace gazebo
