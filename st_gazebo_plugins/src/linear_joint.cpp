#include "linear_joint.hpp"

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

LinearJoint::LinearJoint()
{
}

LinearJoint::~LinearJoint()
{
}

void LinearJoint::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  ros_node_ = gazebo_ros::Node::Get(_sdf);
  joint_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  vel_sub_ = ros_node_->create_subscription<std_msgs::msg::Float32>("device/elevator/target_velocity", 10,
    std::bind(&LinearJoint::TargetVelocityCallback, this, std::placeholders::_1));
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&LinearJoint::OnUpdate, this, std::placeholders::_1));
}

void LinearJoint::Reset(){
}

void LinearJoint::OnUpdate(const gazebo::common::UpdateInfo & _info){
  std::string elevator_joint_name = "elevator_joint";
  auto elevator_joint = model_->GetJoint(elevator_joint_name);
  if (!elevator_joint) {
    RCLCPP_ERROR(
      ros_node_->get_logger(),
      "Joint [%s] not found, plugin will not work.", std::string("elevator_joint").c_str());
    return;
  }


  double pos = elevator_joint->Position();
  double vel = elevator_joint->GetVelocity(0);
  double force = elevator_joint->GetForce(0);

  float vel1 = target_velocity_.data;
  float vel2 = std::min(vel1, (float)((0.45-pos)*0.5));
  float vel3 = std::max(vel2, (float)((-0.05-pos)*0.5));

  float diff = vel - vel3;
  i_term_ += diff * (_info.simTime - last_sim_time_).Float();
  float kp = 10.0;
  float ki = 10.0;
  elevator_joint->SetForce(0, -kp * diff - ki * i_term_);  
  
  last_sim_time_ = _info.simTime;
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  if (seconds_since_last_update < 0.1) {
    return;
  }
  last_update_time_ = _info.simTime;

  // 10Hz

  sensor_msgs::msg::JointState joint_state;
  joint_state.name.push_back(elevator_joint_name);
  joint_state.position.push_back(pos);
  joint_state.velocity.push_back(vel);
  joint_state.effort.push_back(force);
  joint_pub_->publish(joint_state);
}

void LinearJoint::TargetVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg){
  if(msg){
    target_velocity_ = *msg;
  }
}

GZ_REGISTER_MODEL_PLUGIN(LinearJoint)
}  // namespace gazebo_plugins
