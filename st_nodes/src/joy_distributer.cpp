#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyDistibuter : public rclcpp::Node
{
  public:
    JoyDistibuter(): Node("joy_distributer")
    {
      move_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("navigator/move/twist_stamped", 10);
      arm_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/navigator/arm/twist_stamped", 10);
      elevator_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/navigator/elevator/twist_stamped", 10);
      joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyDistibuter::JoyCallback, this, _1));
      timer_ = this->create_wall_timer(500ms, std::bind(&JoyDistibuter::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {
    }
    void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      int next_mode = -1;
      if (msg->buttons[0]) {
        next_mode = 1;
      } else if (msg->buttons[1]) {
        next_mode = 2;
      }

      if(next_mode != -1 && mode_ != (unsigned int)next_mode){
        RCLCPP_INFO(this->get_logger(), "change mode: %u->%u", mode_, next_mode);
        mode_ = next_mode;
      }

      // publish move twist
      geometry_msgs::msg::TwistStamped move_twist;
      move_twist.header.frame_id = "base_link";
      move_twist.header.stamp = msg->header.stamp;
      if(mode_ == 1) {
        move_twist.twist.linear.x = 0.5 * msg->axes[1];
        move_twist.twist.angular.z = 0.5 * msg->axes[3];
      }
      move_pub_->publish(move_twist);

      // publish move twist
      geometry_msgs::msg::TwistStamped arm_twist;
      arm_twist.header.frame_id = "arm_base_link";
      arm_twist.header.stamp = msg->header.stamp;
      if(mode_ == 2) {
        arm_twist.twist.linear.x = 0.2 * msg->axes[1];
        arm_twist.twist.linear.y = 0.2 * msg->axes[0];
      }
      arm_pub_->publish(arm_twist);

      // publish elevator twist
      geometry_msgs::msg::TwistStamped elevator_twist;
      elevator_twist.header.frame_id = "elevator_base_link";
      elevator_twist.header.stamp = msg->header.stamp;
      if(mode_ == 2) {
        elevator_twist.twist.linear.z = 0.1 * msg->axes[4];
      }
      elevator_pub_->publish(elevator_twist);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr move_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr elevator_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    unsigned int mode_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDistibuter>());
  rclcpp::shutdown();
  return 0;
}