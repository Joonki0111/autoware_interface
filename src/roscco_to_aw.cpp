#include "roscco_to_aw/roscco_to_aw.hpp"

namespace roscco_component
{
using namespace std::chrono_literals;
RosccoToAW::RosccoToAW(const rclcpp::NodeOptions & node_options) : Node("roscco_to_aw_node", node_options)
{
    topic_throttle_command_matlab_ = this->create_subscription<std_msgs::msg::Float64>(
        "/brake_command_matlab", rclcpp::QoS(1), std::bind(&RosccoToAW::brakeMLCommandCallback, this,std::placeholders::_1));
    topic_brake_command_matlab_ = this->create_subscription<std_msgs::msg::Float64>(
        "/throttle_command_matlab", rclcpp::QoS(1), std::bind(&RosccoToAW::throttleMLCommandCallback, this,std::placeholders::_1));
    sub_autoware_linear_command_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/trajectory_follower/control_cmd", rclcpp::QoS(1), std::bind(&RosccoToAW::aw_callback, this, std::placeholders::_1));
    sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", rclcpp::QoS(1), std::bind(&RosccoToAW::topic_callback, this, std::placeholders::_1));
    steer_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));        
    velocity_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));      
    velocity_pub_matlab_ = this->create_publisher<std_msgs::msg::Float64>("/vehicle/status/velocity_status_matlab", rclcpp::QoS(1));   
    throttle_cmd_pub_ = this->create_publisher<roscco_msgs::msg::ThrottleCommand>("/throttle_command", rclcpp::QoS(1));       
    brake_cmd_pub_ = this->create_publisher<roscco_msgs::msg::BrakeCommand>("/brake_command", rclcpp::QoS(1));  
    steer_cmd_pub_ = this->create_publisher<roscco_msgs::msg::SteeringCommand>("/steering_command", rclcpp::QoS(1));  
    velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/velocity_command", rclcpp::QoS(1));  
    timer_ = this->create_wall_timer(50ms, std::bind(&RosccoToAW::timer_callback, this));
}

void RosccoToAW::topic_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id== 688) 
    {
      double steering_angle_report = msg->data[0] + (msg->data[1] << 8);
      if(steering_angle_report > 60000) 
      {
          steering_angle_report -= 65535;
      }
      angle = steering_angle_report;
      angle *= DEG2RAD;
      angle /= 10.0;
      angle += 0.06;
      steer_msg_.stamp = msg->header.stamp;
      steer_msg_.steering_tire_angle = (angle/15.7);
      steer_pub_->publish(steer_msg_);
    }    

    if(msg->id== 1200)  //0x4B0 = 1200 
    {
      speed_report = 0.5 * ((msg->data[4] + msg->data[5] * 256) + (msg->data[6] + msg->data[7] * 256));
      speed_report *= WHEEL_SPEED_RATIO;

      speed_report *= direction;
      velocity_msg_.header.stamp = msg->header.stamp;
      velocity_msg_.header.frame_id = "base_link";
      velocity_msg_.longitudinal_velocity = speed_report * KPH2MPS;
      velocity_msg_.heading_rate = ((speed_report * KPH2MPS)*std::tan(steer_msg_.steering_tire_angle))/SOUL_WHEEL_BASE;
      velocity_pub_->publish(velocity_msg_);

      velocity_matlab_msg.data = speed_report * KPH2MPS;
      velocity_pub_matlab_->publish(velocity_matlab_msg);
    }

    if(msg->id== 657) //0x291 = 657
    {
      double motor_direction = msg->data[2] + msg->data[3] * 256;

      if(motor_direction > 60000) 
      {
        motor_direction -= 65535;
      }

      if(motor_direction < 0)
      {
        direction = -1;
      }
      else if(motor_direction > 0)
      {
        direction = 1;
      }
      else
      {
        direction = direction;
      }
    }
}

void RosccoToAW::throttleMLCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  throttle = msg->data;
}
void RosccoToAW::brakeMLCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  brake = msg->data;
}

void RosccoToAW::timer_callback()
{
    roscco_throttle_msg.throttle_position = throttle;
    roscco_brake_msg.brake_position = brake;
    roscco_steering_msg.steering_torque = 0.0;
    throttle_cmd_pub_->publish(roscco_throttle_msg);
    brake_cmd_pub_->publish(roscco_brake_msg);
    // steer_cmd_pub_->publish(roscco_steering_msg);
}
void RosccoToAW::aw_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    velocity_command = msg->longitudinal.speed;
    velocity_command_msg.data = velocity_command;
    velocity_cmd_pub_->publish(velocity_command_msg);
}

} // namespace roscco_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(roscco_component::RosccoToAW)
