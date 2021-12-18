#include "arm_joint.hpp"

#include <sdf/Element.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Distortion.hh>
#include <gazebo/sensors/SensorTypes.hh>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <ignition/math/Helpers.hh>

#include <camera_info_manager/camera_info_manager.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#define FLOAT_SIZE sizeof(float)

namespace gazebo_plugins
{

ArmJoint::ArmJoint()
{
}

ArmJoint::~ArmJoint()
{
}

void ArmJoint::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  ros_node_ = gazebo_ros::Node::Get(_sdf);
  joint_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  vel_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32MultiArray>("device/arm/target_velocity_list", 10,
    std::bind(&ArmJoint::TargetVelocityCallback, this, std::placeholders::_1));
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ArmJoint::OnUpdate, this, std::placeholders::_1));
}

void ArmJoint::Reset(){
}

void ArmJoint::OnUpdate(const gazebo::common::UpdateInfo & _info){

  std::vector<std::string> arm_joint_names = {"arm1_joint", "arm2_joint", "arm3_joint"};
  std::vector<gazebo::physics::JointPtr> arm_joints;
  for (auto joint_name : arm_joint_names){
    auto arm_joint = model_->GetJoint(joint_name);
    if (!arm_joint) {
      RCLCPP_ERROR(ros_node_->get_logger(), "[%s] not found", joint_name.c_str());
      return;
    }
    arm_joints.push_back(arm_joint);
  }

  for (size_t i = 0; i < 3; i++) {
    double pos = arm_joints[i]->Position();
    double vel = arm_joints[i]->GetVelocity(0);

    float vel1 = target_velocity_[i];
    float vel2 = std::min(vel1, (float)((2.5-pos)*2.0));
    float vel3 = std::max(vel2, (float)((-2.5-pos)*2.0));

    float diff = vel - vel3;
    i_term_[i] += diff * (_info.simTime - last_sim_time_).Float();
    float kp = 0.5;
    float ki = 0.5;
    float output = -kp * diff - ki * i_term_[i];
    arm_joints[i]->SetForce(0, output);
  }

  last_sim_time_ = _info.simTime;
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  if (seconds_since_last_update < 0.1) {
    return;
  }
  last_update_time_ = _info.simTime;

  // 10Hz
  // telemetry
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_info.simTime);
  for (size_t i = 0; i < 3; i++) {
    joint_state.name.push_back(arm_joint_names[i]);
    double pos = arm_joints[i]->Position();
    double vel = arm_joints[i]->GetVelocity(0);
    double force = arm_joints[i]->GetForce(0);
    joint_state.position.push_back(pos);
    joint_state.velocity.push_back(vel);
    joint_state.effort.push_back(force);
  }
  joint_pub_->publish(joint_state);
}

void ArmJoint::TargetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
  if(msg){
    for(size_t i = 0; i < std::min((size_t)3, msg->data.size()); i++){
      target_velocity_[i] = msg->data[i];
    }
  }
}

GZ_REGISTER_MODEL_PLUGIN(ArmJoint)
}  // namespace gazebo_plugins
