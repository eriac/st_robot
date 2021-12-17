#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyDistibuter : public rclcpp::Node
{
  public:
    JoyDistibuter(): Node("arm_navigator")
    {
      elevator_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>("device/elevator/target_velocity", 10);
      arm_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("device/arm/target_velocity_list", 10);
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("navigator/arm/twist_stamped", 10, std::bind(&JoyDistibuter::TwistCallback, this, _1));
      timer_ = this->create_wall_timer(100ms, std::bind(&JoyDistibuter::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {
      // twist_pub_->publish(input_.twist);
      std_msgs::msg::Float32MultiArray float_list;
      float_list.data.push_back(input_.twist.linear.x);
      float_list.data.push_back(input_.twist.linear.y);
      float_list.data.push_back(input_.twist.linear.z);
      arm_vel_pub_->publish(float_list);
    }

    void TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      input_ = *msg;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;

    geometry_msgs::msg::TwistStamped input_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDistibuter>());
  rclcpp::shutdown();
  return 0;
}