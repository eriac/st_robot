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
      arm_vel_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("device/arm/target_velocity_list", 10);
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("navigator/arm/twist_stamped", 10, std::bind(&JoyDistibuter::TwistCallback, this, _1));
      joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&JoyDistibuter::JointStateCallback, this, _1));
      timer_ = this->create_wall_timer(100ms, std::bind(&JoyDistibuter::TimerCallback, this));
    }

  private:
    void TimerCallback()
    {
      float l = 0.2;
      float th1 = joint_position_[0];
      float th2 = joint_position_[1];
      float th3 = joint_position_[2];
      float a = -l*sin(th1)-l*sin(th1+th2);
      float b = -l*sin(th1+th2);
      float c = l*cos(th1)+l*cos(th1+th2);
      float d = l*cos(th1+th2);
      float D = a*d-b*c;
      if (fabsf(D)<0.001){
        if(0<D)D=0.001;
        else D=-0.001;
      }

      float dx = input_.twist.linear.x;
      float dy = input_.twist.linear.y;
      float dth1 = (1/D)*( d*dx-b*dy);
      float dth2 = (1/D)*(-c*dx+a*dy);

      // th2 < -10[deg]
      float max_th2 = -10 *M_PI/180; 
      float max_dth2 = (max_th2-th2)*0.5;
      dth2 = std::min(dth2, max_dth2);

      // arm3 must point left side
      float target_th3 = -M_PI/2 - (th1+th2);
      float dth3 = -2.0*(th3 - target_th3) - dth1 - dth2;

      std_msgs::msg::Float32MultiArray float_list;
      float_list.data.push_back(dth1);
      float_list.data.push_back(dth2);
      float_list.data.push_back(dth3);
      arm_vel_pub_->publish(float_list);
    }

    void TwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
      input_ = *msg;
    }

    void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      size_t data_size = std::min(msg->name.size(), msg->position.size());
      for(size_t i = 0; i < data_size; i++){
        if(msg->name[i] == "arm1_joint"){
          joint_position_[0] = msg->position[i];
        } else if(msg->name[i] == "arm2_joint"){
          joint_position_[1] = msg->position[i];
        } else if(msg->name[i] == "arm3_joint"){
          joint_position_[2] = msg->position[i];
        }
      }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr arm_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    geometry_msgs::msg::TwistStamped input_;
    float joint_position_[3]{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyDistibuter>());
  rclcpp::shutdown();
  return 0;
}