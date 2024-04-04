#include "roscco_to_aw/roscco_to_aw.hpp"

namespace roscco_component
{
using namespace std::chrono_literals;
RosccoToAW::RosccoToAW(const rclcpp::NodeOptions & node_options) : Node("roscco_to_aw_node", node_options)
{
    sub_throttle_command_matlab_ = this->create_subscription<std_msgs::msg::Float64>(
        "/brake_command_matlab", rclcpp::QoS(1), std::bind(&RosccoToAW::brakeMLCommandCallback, this,std::placeholders::_1)); // from matlab
    sub_brake_command_matlab_ = this->create_subscription<std_msgs::msg::Float64>(
        "/throttle_command_matlab", rclcpp::QoS(1), std::bind(&RosccoToAW::throttleMLCommandCallback, this,std::placeholders::_1)); // from matlab
    sub_steering_command_matlab_ = this->create_subscription<std_msgs::msg::Float64>(
        "/steering_command_matlab", rclcpp::QoS(1), std::bind(&RosccoToAW::steeringMLCommandCallback, this,std::placeholders::_1)); // from matlab

    sub_autoware_command_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/trajectory_follower/control_cmd", rclcpp::QoS(1), std::bind(&RosccoToAW::aw_callback, this, std::placeholders::_1)); // from autoware

    sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", rclcpp::QoS(1), std::bind(&RosccoToAW::topic_callback, this, std::placeholders::_1)); // from ros2_socketcan

    steer_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1)); // to autoware   
    velocity_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1)); // to autoware         

    velocity_pub_matlab_ = this->create_publisher<std_msgs::msg::Float64>("/vehicle/status/velocity_status_matlab", rclcpp::QoS(1)); // to matlab     
    steer_pub_matlab_ = this->create_publisher<std_msgs::msg::Float64>("/vehicle/status/steering_status_matlab", rclcpp::QoS(1)); // to matlab    

    throttle_cmd_roscco_pub_ = this->create_publisher<roscco_msgs::msg::ThrottleCommand>("/throttle_command", rclcpp::QoS(1)); // to ros_to_oscc   
    brake_cmd_roscco_pub_ = this->create_publisher<roscco_msgs::msg::BrakeCommand>("/brake_command", rclcpp::QoS(1)); // to ros_to_oscc  
    steer_cmd_roscco_pub_ = this->create_publisher<roscco_msgs::msg::SteeringCommand>("/steering_command", rclcpp::QoS(1)); // to ros_to_oscc

    velocity_cmd_matlab_pub_ = this->create_publisher<std_msgs::msg::Float64>("/velocity_command_aw", rclcpp::QoS(1)); // to matlab 
    steer_cmd_matlab_pub_ = this->create_publisher<std_msgs::msg::Float64>("/steering_command_aw", rclcpp::QoS(1)); // to matlab 

    timer_ = this->create_wall_timer(10ms, std::bind(&RosccoToAW::timer_callback, this)); 
}

void RosccoToAW::topic_callback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if(msg->id== 688) //0x2B0
  {
    double steering_angle_report = msg->data[0] + (msg->data[1] << 8);
    if(steering_angle_report > 60000) 
    {
        steering_angle_report -= 65535;
    }
    double angle = steering_angle_report;
    angle *= DEG2RAD;
    angle /= 10.0;
    angle += 0.06;
    steer_msg_.stamp = msg->header.stamp;
    steer_msg_.steering_tire_angle = (angle/15.7);
    steer_pub_->publish(steer_msg_);

    steer_matlab_msg.data = angle;
    steer_pub_matlab_->publish(steer_matlab_msg);
  }    

  if(msg->id== 657) //0x291
  {
    double motor_revolution = msg->data[2] + msg->data[3] * 256; //RPM

    if(motor_revolution > 60000) 
    {
      motor_revolution -= 65535;
    }

    double speed_report = motor_revolution / 2.1 * WHEEL_SPEED_RATIO;

    velocity_msg_.header.stamp = msg->header.stamp;
    velocity_msg_.header.frame_id = "base_link";

    velocity_msg_.longitudinal_velocity = speed_report * KPH2MPS;
    velocity_matlab_msg.data = speed_report * KPH2MPS;

    velocity_msg_.heading_rate = ((speed_report * KPH2MPS)*std::tan(steer_msg_.steering_tire_angle))/SOUL_WHEEL_BASE;

    velocity_pub_->publish(velocity_msg_);
    velocity_pub_matlab_->publish(velocity_matlab_msg);
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
void RosccoToAW::steeringMLCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  steering = msg->data;
}

void RosccoToAW::timer_callback()
{
    roscco_throttle_msg.throttle_position = throttle;
    roscco_brake_msg.brake_position = brake;
    roscco_steering_msg.steering_torque = steering;
    throttle_cmd_roscco_pub_->publish(roscco_throttle_msg);
    brake_cmd_roscco_pub_->publish(roscco_brake_msg);
    steer_cmd_roscco_pub_->publish(roscco_steering_msg);
}

void RosccoToAW::aw_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    float velocity_command = msg->longitudinal.speed;
    float steer_command = msg->lateral.steering_tire_angle;
    
    velocity_command_msg.data = velocity_command;
    steer_command_msg.data = steer_command*15.7;
    
    velocity_cmd_matlab_pub_->publish(velocity_command_msg);
    steer_cmd_matlab_pub_->publish(steer_command_msg);
}

} // namespace roscco_component
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(roscco_component::RosccoToAW)
