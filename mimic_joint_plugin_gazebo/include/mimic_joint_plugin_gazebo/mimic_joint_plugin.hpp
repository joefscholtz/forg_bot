#ifndef GAZEBO_PLUGINS__MIMIC_JOINT_PLUGIN_HPP_
#define GAZEBO_PLUGINS__MIMIC_JOINT_PLUGIN_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/JointController.hh>
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

  void OnUpdate(const gazebo::common::UpdateInfo &_info);

private:
  bool debug, use_pid;

  std::string joint_name_, mimic_joint_name_, mimic_joint_scoped_name;

  double multiplier_, offset_;

  gazebo::physics::JointPtr joint_, mimic_joint_;

  gazebo::physics::ModelPtr model_;

  gazebo::physics::WorldPtr world_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  gazebo::common::Time last_update_time_;

  double update_period_; // in seconds

  physics::JointControllerPtr jointController;
  double kp, ki, kd;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)
} // namespace gazebo

#endif // GAZEBO_PLUGINS__MIMIC_JOINT_PLUGIN_HPP_
