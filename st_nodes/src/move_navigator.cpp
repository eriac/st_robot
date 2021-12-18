#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class JoyDistibuter : public rclcpp::Node
{
  public:
    JoyDistibuter(): Node("move_navigator")
    {
      twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("device/move/cmd_vel", 10);
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("navigator/move/twist_stamped", 10, std::bind(&JoyDistibuter::TwistCallback, this, _1));
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("device/move/odom", 10, std::bind(&JoyDistibuter::OdomCallback, this, _1));
      timer_ = this->create_wall_timer(100ms, std::bind(&JoyDistibuter::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {
      twist_pub_->publish(input_.twist);

      // if(0.05 < fabsf(odom_.twist.twist.linear.x) || 0.02<fabsf(odom_.twist.twist.angular.z)){
      //    RCLCPP_INFO(this->get_logger(), "active");
      // }
      //   rclcpp::Time now =  get_clock()->now();
    }

    void TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      input_ = *msg;
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      odom_ = *msg;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::TwistStamped input_;
    nav_msgs::msg::Odometry odom_;
    rclcpp::Time last_active_stamp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDistibuter>());
  rclcpp::shutdown();
  return 0;
}