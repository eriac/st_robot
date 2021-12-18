#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyDistibuter : public rclcpp::Node
{
  public:
    JoyDistibuter(): Node("arm_navigator")
    {
      elevator_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>("device/elevator/target_velocity", 10);
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("navigator/elevator/twist_stamped", 10, std::bind(&JoyDistibuter::TwistCallback, this, _1));
      joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JoyDistibuter::JointStateCallback, this, _1));
      timer_ = this->create_wall_timer(100ms, std::bind(&JoyDistibuter::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {

      std_msgs::msg::Float32 float_msg;
      float_msg.data = input_.twist.linear.z;
      elevator_vel_pub_->publish(float_msg);
    }

    void TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      input_ = *msg;
    }

    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      size_t data_size = std::min(msg->name.size(), msg->position.size());
      for(size_t i = 0; i < data_size; i++){
        if(msg->name[i] == "elevator_joint"){
          elevator_position_ = msg->position[i];
        }
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    geometry_msgs::msg::TwistStamped input_;
    float elevator_position_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDistibuter>());
  rclcpp::shutdown();
  return 0;
}