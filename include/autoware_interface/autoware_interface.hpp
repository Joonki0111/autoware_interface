#ifndef AUTOWARE_INTERFACE__AUTOWARE_INTERFACE_HPP_
#define AUTOWARE_INTERFACE__AUTOWARE_INTERFACE_HPP_

#include <memory>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "roscco_msgs/msg/brake_command.hpp"
#include "roscco_msgs/msg/throttle_command.hpp"
#include "roscco_msgs/msg/steering_command.hpp"
#include "roscco_msgs/msg/roscco_status.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "can_msgs/msg/frame.hpp"


#define KPH2MPS 1/3.6
#define SOUL_WHEEL_BASE 2.57048
#define DEG2RAD 0.0174533
#define WHEEL_SPEED_RATIO 0.03125

namespace autoware_interface_ns
{
class AutowareInterface : public rclcpp::Node
{
    public:
        explicit AutowareInterface(const rclcpp::NodeOptions & node_options);

    private:
        struct ROSCCOStatus
        {
            bool brake_enabled = false;
            bool steer_enabled = false;
            bool throttle_enabled = false;
        };

        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr vehicle_CAN_sub_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr ROSCCO_CAN_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_throttle_command_sub_; 
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_brake_command_sub_; 
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr TC_steer_command_sub_; 
        rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr AW_command_sub_;
        
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_steer_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_velocity_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr TC_steer_cmd_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr TC_ROSCCO_status_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>::SharedPtr AW_velocity_status_pub_;
        rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr AW_steer_status_pub_;
        rclcpp::Publisher<roscco_msgs::msg::ThrottleCommand>::SharedPtr ROSCCO_throttle_cmd_pub_;
        rclcpp::Publisher<roscco_msgs::msg::BrakeCommand>::SharedPtr ROSCCO_brake_cmd_pub_;
        rclcpp::Publisher<roscco_msgs::msg::SteeringCommand>::SharedPtr ROSCCO_steer_cmd_pub_;
        rclcpp::Publisher<roscco_msgs::msg::RosccoStatus>::SharedPtr ROSCCO_status_pub_;

        rclcpp::TimerBase::SharedPtr timer_;
        
        ROSCCOStatus roscco_status_{};
        double TC_throttle_cmd_ = 0.0; 
        double TC_brake_cmd_ = 0.0; 
        double TC_steer_cmd_ = 0.0; 
        double AW_velocity_command_ = 0.0;
        double AW_steer_command_ = 0.0;
        double steering_angle_ = 0.0;
        double velocity_ = 0.0;

        void VehicleCANCallback(const can_msgs::msg::Frame::SharedPtr msg);
        void ROSCCOCANCallback(const can_msgs::msg::Frame::SharedPtr msg);
        void TCthrottlecmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void TCbrakecmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void TCsteercmdCallback(const std_msgs::msg::Float64::SharedPtr msg);
        void AWcmdcallback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
        void TimerCallback();
};
}
#endif  // AUTOWARE_INTERFACE__AUTOWARE_INTERFACE_HPP_
