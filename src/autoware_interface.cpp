#include "autoware_interface/autoware_interface.hpp"

namespace autoware_interface_ns
{

using namespace std::chrono_literals;

AutowareInterface::AutowareInterface(const rclcpp::NodeOptions & node_options) : Node("roscco_to_aw_node", node_options)
{
    vehicle_CAN_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/socketcan/vehicle/from_can_bus", rclcpp::QoS(1), std::bind(&AutowareInterface::VehicleCANCallback, this, std::placeholders::_1));
    ROSCCO_CAN_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/socketcan/roscco/from_can_bus", rclcpp::QoS(1), std::bind(&AutowareInterface::ROSCCOCANCallback, this, std::placeholders::_1));
    TC_throttle_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/throttle_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCthrottlecmdCallback, this,std::placeholders::_1));
    TC_brake_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/brake_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCbrakecmdCallback, this,std::placeholders::_1));
    TC_steer_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "/twist_controller/output/steering_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::TCsteercmdCallback, this,std::placeholders::_1));
    AW_command_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
        "/control/command/control_cmd", rclcpp::QoS(1), std::bind(&AutowareInterface::AWcmdcallback, this, std::placeholders::_1));

    TC_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_status", rclcpp::QoS(1));
    TC_steer_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steering_status", rclcpp::QoS(1));   
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_cmd", rclcpp::QoS(1)); 
    TC_steer_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steering_cmd", rclcpp::QoS(1));
    TC_ROSCCO_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/roscco_status", rclcpp::QoS(1));
    AW_velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    AW_steer_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));  
    ROSCCO_throttle_cmd_pub_ = this->create_publisher<roscco_msgs::msg::ThrottleCommand>("/roscco/throttle_cmd", rclcpp::QoS(1));  
    ROSCCO_brake_cmd_pub_ = this->create_publisher<roscco_msgs::msg::BrakeCommand>("/roscco/brake_cmd", rclcpp::QoS(1));
    ROSCCO_steer_cmd_pub_ = this->create_publisher<roscco_msgs::msg::SteeringCommand>("/roscco/steering_cmd", rclcpp::QoS(1));
    ROSCCO_status_pub_ = this->create_publisher<roscco_msgs::msg::RosccoStatus>("/roscco/status", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(10ms, std::bind(&AutowareInterface::TimerCallback, this));
}

void AutowareInterface::VehicleCANCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if(msg->id== 688) //0x2B0
    {
        double steering_angle_report = msg->data[0] + (msg->data[1] << 8);
        if(steering_angle_report > 60000) 
        {
            steering_angle_report -= 65535;
        }

        steering_angle_ = steering_angle_report;
        steering_angle_ *= DEG2RAD;
        steering_angle_ /= 10.0;
    }    

    if(msg->id== 657) //0x291
    {
        double motor_revolution = msg->data[2] + msg->data[3] * 256; //RPM

        if(motor_revolution > 60000) 
        {
            motor_revolution -= 65535;
        }

        velocity_ = motor_revolution / 2.1 * WHEEL_SPEED_RATIO;
    }
}
void AutowareInterface::ROSCCOCANCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    const uint32_t can_id = msg->id;

    switch (can_id)
    {
        case 115:
            roscco_status_.brake_enabled = msg->data[2];
            break;
        case 131:
            roscco_status_.steer_enabled = msg->data[2];
            break;
        case 147:
            roscco_status_.throttle_enabled = msg->data[2];
            break;
        default:
            break;
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

void AutowareInterface::AWcmdcallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
    AW_velocity_command_ = msg->longitudinal.speed;
    AW_steer_command_ = msg->lateral.steering_tire_angle;
}

void AutowareInterface::TimerCallback()
{    
    // To Autoware
    autoware_auto_vehicle_msgs::msg::VelocityReport AW_velocity_status_msg;
    autoware_auto_vehicle_msgs::msg::SteeringReport AW_steering_tire_status_msg;

    AW_velocity_status_msg.header.stamp = this->now();
    AW_velocity_status_msg.header.frame_id = "base_link";
    AW_velocity_status_msg.longitudinal_velocity = velocity_ * KPH2MPS;
    AW_velocity_status_msg.heading_rate = ((velocity_ * KPH2MPS) * std::tan(steering_angle_ / 15.7)) / SOUL_WHEEL_BASE;
    AW_steering_tire_status_msg.stamp = this->now();
    AW_steering_tire_status_msg.steering_tire_angle = (steering_angle_ / 15.7);

    AW_velocity_status_pub_->publish(AW_velocity_status_msg);
    AW_steer_status_pub_->publish(AW_steering_tire_status_msg);



    // To Roscco
    roscco_msgs::msg::ThrottleCommand ROSCCO_throttle_msg;
    roscco_msgs::msg::BrakeCommand ROSCCO_brake_msg;
    roscco_msgs::msg::SteeringCommand ROSCCO_steering_msg;

    ROSCCO_throttle_msg.throttle_position = TC_throttle_cmd_;
    ROSCCO_brake_msg.brake_position = TC_brake_cmd_;
    ROSCCO_steering_msg.steering_torque = TC_steer_cmd_;

    ROSCCO_throttle_cmd_pub_->publish(ROSCCO_throttle_msg);
    ROSCCO_brake_cmd_pub_->publish(ROSCCO_brake_msg);
    ROSCCO_steer_cmd_pub_->publish(ROSCCO_steering_msg);
    


    // To TwistController
    std_msgs::msg::Float64 TC_velocity_command_msg;
    std_msgs::msg::Float64 TC_velocity_status_msg;
    std_msgs::msg::Float64 TC_steer_command_msg;
    std_msgs::msg::Float64 TC_steer_status_msg;
    std_msgs::msg::Bool TC_roscco_status_msg;

    TC_velocity_command_msg.data = AW_velocity_command_;
    TC_velocity_status_msg.data = velocity_ * KPH2MPS;
    TC_steer_command_msg.data = AW_steer_command_ * 15.7;
    TC_steer_status_msg.data = steering_angle_;
    if(roscco_status_.brake_enabled + roscco_status_.steer_enabled + roscco_status_.throttle_enabled == 3)
    {
        TC_roscco_status_msg.data = true;
    }
    else
    {
        TC_roscco_status_msg.data = false;
    }

    TC_velocity_cmd_pub_->publish(TC_velocity_command_msg);
    TC_velocity_status_pub_->publish(TC_velocity_status_msg);
    TC_steer_cmd_pub_->publish(TC_steer_command_msg);
    TC_steer_status_pub_->publish(TC_steer_status_msg);
    TC_ROSCCO_status_pub_->publish(TC_roscco_status_msg);
    


    roscco_msgs::msg::RosccoStatus roscco_status_msg;
    roscco_status_msg.brake_status = roscco_status_.brake_enabled;
    roscco_status_msg.steering_status = roscco_status_.steer_enabled;
    roscco_status_msg.throttle_status = roscco_status_.throttle_enabled;
    ROSCCO_status_pub_->publish(roscco_status_msg);
}
} // namespace autoware_interface_ns
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware_interface_ns::AutowareInterface)
