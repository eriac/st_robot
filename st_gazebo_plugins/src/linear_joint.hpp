#ifndef ST_GAZEBO_PLUGINS__LINEAR_JOINT_HPP_
#define ST_GAZEBO_PLUGINS__LINEAR_JOINT_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo_ros/node.hpp>
#include <memory>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

namespace gazebo_plugins
{

class LinearJoint : public gazebo::ModelPlugin
{
public:
  /// Constructor
  LinearJoint();

  /// Destructor
  ~LinearJoint();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

  // Documentation inherited
  void Reset() override;

  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  void TargetVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg);

private:
  gazebo_ros::Node::SharedPtr ros_node_;
  gazebo::physics::ModelPtr model_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_;
  std_msgs::msg::Float32 target_velocity_;
  gazebo::event::ConnectionPtr update_connection_;

  /// Last update time
  gazebo::common::Time last_update_time_;
};
}  // namespace gazebo_plugins

#endif  // ST_GAZEBO_PLUGINS__LINEAR_JOINT_HPP_
