#include "mimic_joint_plugin_gazebo/mimic_joint_plugin.hpp"

namespace gazebo {

MimicJointPlugin::MimicJointPlugin() {}

MimicJointPlugin::~MimicJointPlugin() {}

// Inherited
void MimicJointPlugin::Load(gazebo::physics::ModelPtr _model,
                            sdf::ElementPtr _sdf) {
  model_ = _model;
  world_ = model_->GetWorld();

  if (!model_) {
    std::cerr << "\nParent model is NULL! MimicJointPlugin could not be loaded."
              << std::endl;
    return;
  }

  if (_sdf->HasElement("namespace")) {
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
    // prevent exception: namespace must be absolute, it must lead with a '/'
    if (namespace_.empty() || namespace_[0] != '/') {
      namespace_ = '/' + namespace_;
    }
  }
}

// Inherited
void MimicJointPlugin::Reset() {}

} // namespace gazebo
