#include "autoware_interface/autoware_interface.hpp"

namespace autoware_interface_ns
{
using namespace std::chrono_literals;

AutowareInterface::AutowareInterface(const rclcpp::NodeOptions & node_options) : Node("autoware_interface", node_options)
{
    TC_throttle_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/brake_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCthrottlecmdCallback, this,std::placeholders::_1));
    TC_brake_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/throttle_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCbrakecmdCallback, this,std::placeholders::_1));
    TC_steering_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/steering_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCsteercmdCallback, this,std::placeholders::_1));
    autoware_command_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWcmdcallback, this, std::placeholders::_1));
    CAN_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/from_can_bus", rclcpp::QoS(1), std::bind(&AutowareInterface::CANCallback, this, std::placeholders::_1));

    TC_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_status", rclcpp::QoS(1));
    TC_steering_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steering_status", rclcpp::QoS(1));   
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_cmd", rclcpp::QoS(1)); 
    TC_steer_cmd__pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steering_cmd", rclcpp::QoS(1));
    velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    steer_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));  
    roscco_throttle_cmd_pub_ = this->create_publisher<roscco_msgs::msg::ThrottleCommand>("/roscco/throttle_cmd", rclcpp::QoS(1));  
    roscco_brake_cmd_pub_ = this->create_publisher<roscco_msgs::msg::BrakeCommand>("/roscco/brake_cmd", rclcpp::QoS(1));
    roscco_steer_cmd_pub_ = this->create_publisher<roscco_msgs::msg::SteeringCommand>("/roscco/steering_cmd", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(10ms, std::bind(&AutowareInterface::timer_callback, this)); 
}

void AutowareInterface::CANCallback(const can_msgs::msg::Frame::SharedPtr msg)
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
        AW_steering_status_msg_.stamp = msg->header.stamp;
        AW_steering_status_msg_.steering_tire_angle = (angle/15.7);
        steer_status_pub_->publish(AW_steering_status_msg_);

        TC_steer_status_msg_.data = angle;
        TC_steering_status_pub_->publish(TC_steer_status_msg_);
    }    

    if(msg->id== 657) //0x291
    {
        double motor_revolution = msg->data[2] + msg->data[3] * 256; //RPM

        if(motor_revolution > 60000) 
        {
            motor_revolution -= 65535;
        }

        double speed_report = motor_revolution / 2.1 * WHEEL_SPEED_RATIO;

        AW_velocity_status_msg_.header.stamp = msg->header.stamp;
        AW_velocity_status_msg_.header.frame_id = "base_link";

        AW_velocity_status_msg_.longitudinal_velocity = speed_report * KPH2MPS;
        TC_velocity_status_msg_.data = speed_report * KPH2MPS;

        AW_velocity_status_msg_.heading_rate = ((speed_report * KPH2MPS)*std::tan(AW_steering_status_msg_.steering_tire_angle))/SOUL_WHEEL_BASE;

        velocity_status_pub_->publish(AW_velocity_status_msg_);
        TC_velocity_status_pub_->publish(TC_velocity_status_msg_);
    }
}

void AutowareInterface::TCthrottlecmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    TC_throttle_cmd_ = msg->data;
}
void AutowareInterface::TCbrakecmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    TC_brake_cmd_ = msg->data;
}
void AutowareInterface::TCsteercmdCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    TC_steer_cmd_ = msg->data;
}

void AutowareInterface::timer_callback()
{
    roscco_throttle_msg.throttle_position = TC_throttle_cmd_;
    roscco_brake_msg.brake_position = TC_brake_cmd_;
    roscco_steering_msg.steering_torque = TC_steer_cmd_;

    roscco_throttle_cmd_pub_->publish(roscco_throttle_msg);
    roscco_brake_cmd_pub_->publish(roscco_brake_msg);
    roscco_steer_cmd_pub_->publish(roscco_steering_msg);
    
    steer_status_pub_->publish(AW_steering_status_msg_);
    TC_steering_status_pub_->publish(TC_steer_status_msg_);
}

void AutowareInterface::AWcmdcallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    float velocity_command = msg->longitudinal.speed;
    float steer_command = msg->lateral.steering_tire_angle;
    
    TC_velocity_command_msg_.data = velocity_command;
    TC_steer_matlab_msg_.data = steer_command*15.7;
    
    TC_velocity_cmd_pub_->publish(TC_velocity_command_msg_);
    TC_steer_cmd__pub_->publish(TC_steer_matlab_msg_);
}

} // namespace autoware_interface_ns
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_interface_ns::AutowareInterface)
