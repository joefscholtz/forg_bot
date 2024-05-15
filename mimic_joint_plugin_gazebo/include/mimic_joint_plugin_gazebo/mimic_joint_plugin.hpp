#ifndef GAZEBO_PLUGINS__MIMIC_JOINT_PLUGIN_HPP_
#define GAZEBO_PLUGINS__MIMIC_JOINT_PLUGIN_HPP_

#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

namespace gazebo {

/**
  Example Usage:
  \code{.xml}
    <plugin name="name" filename="libgazebo_mimic_joint_plugin.so">

    </plugin>
  \endcode
*/

class MimicJointPlugin : public gazebo::ModelPlugin {
public:
  MimicJointPlugin();

  ~MimicJointPlugin();

protected:
  // Inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Inherited
  void Reset() override;

private:
  std::string joint_name_, mimic_joint_name_, namespace_;

  double multiplier_, offset_;

  physics::JointPtr joint_, mimic_joint_;

  physics::ModelPtr model_;

  physics::WorldPtr world_;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)
} // namespace gazebo

#endif // GAZEBO_PLUGINS__MIMIC_JOINT_PLUGIN_HPP_
