#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyDistibuter : public rclcpp::Node
{
  public:
    JoyDistibuter(): Node("move_navigator")
    {
      twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("device/move/cmd_vel", 10);
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("navigator/move/twist_stamped", 10, std::bind(&JoyDistibuter::TwistCallback, this, _1));
      timer_ = this->create_wall_timer(100ms, std::bind(&JoyDistibuter::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {
      twist_pub_->publish(input_.twist);
    }

    void TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      input_ = *msg;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
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