#include "autoware_interface/autoware_interface.hpp"

using namespace std::chrono_literals;

AutowareInterface::AutowareInterface() : Node("autoware_interface")
{
    alive_clock_.roscco = get_clock()->now();
    alive_clock_.adma = get_clock()->now();
    alive_clock_.os = get_clock()->now();
    alive_clock_.tc = get_clock()->now();
    alive_clock_.roscco_can = get_clock()->now();
    alive_clock_.vehicle_can = get_clock()->now();

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
    AW_mode_sub_ = this->create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
        "/control/vehicle_cmd_gate/operation_mode", rclcpp::QoS(1), std::bind(&AutowareInterface::AWmodecallback, this, std::placeholders::_1));
    TC_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/twist_controller/output/clock", rclcpp::QoS(1), std::bind(&AutowareInterface::TCclockCallback, this,std::placeholders::_1)); //250304 JSJ
    ROSCCO_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/roscco/clock", rclcpp::QoS(1), std::bind(&AutowareInterface::ROSCCOclockCallback, this,std::placeholders::_1));
    Ouster_clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/sensing/ouster/clock", rclcpp::QoS(1), std::bind(&AutowareInterface::OusterclockCallback, this,std::placeholders::_1));
    ADMA_clock_sub_ = this->create_subscription<adma_ros_driver_msgs::msg::AdmaDataScaled>(
        "/sensing/genesys/adma/data_scaled", rclcpp::QoS(1), std::bind(&AutowareInterface::ADMAclockCallback, this,std::placeholders::_1));

    TC_velocity_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_status", rclcpp::QoS(1));
    TC_steer_status_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steering_status", rclcpp::QoS(1));   
    TC_velocity_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/velocity_cmd", rclcpp::QoS(1)); 
    TC_steer_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("/twist_controller/input/steering_cmd", rclcpp::QoS(1));
    TC_ROSCCO_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/twist_controller/roscco/status", rclcpp::QoS(1));
    AW_velocity_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>(
        "/vehicle/status/velocity_status", rclcpp::QoS(1));
    AW_steer_status_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>(
        "/vehicle/status/steering_status", rclcpp::QoS(1));  
    ROSCCO_throttle_cmd_pub_ = this->create_publisher<roscco_msgs::msg::ThrottleCommand>("/roscco/throttle_cmd", rclcpp::QoS(1));  
    ROSCCO_brake_cmd_pub_ = this->create_publisher<roscco_msgs::msg::BrakeCommand>("/roscco/brake_cmd", rclcpp::QoS(1));
    ROSCCO_steer_cmd_pub_ = this->create_publisher<roscco_msgs::msg::SteeringCommand>("/roscco/steering_cmd", rclcpp::QoS(1));
    ROSCCO_status_pub_ = this->create_publisher<roscco_msgs::msg::RosccoStatus>("/roscco/status", rclcpp::QoS(1));
    autoware_control_pub_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::ControlModeReport>(
        "/vehicle/status/control_mode", rclcpp::QoS(1));
    component_status_pub_ = this->create_publisher<autoware_system_msgs::msg::ComponentStatus>(
        "/system/status/component_status", rclcpp::QoS(1));
    clock_pub = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1); //HJK_250311_A
    steer_aligned_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vehicle/steer_aligned_status", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(10ms, std::bind(&AutowareInterface::TimerCallback, this));
    clock_timer_ = this->create_wall_timer(100ms, std::bind(&AutowareInterface::ClockTimerCallback, this));

    AW_stop_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>("/api/operation_mode/change_to_stop");
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
    alive_clock_.vehicle_can = msg->header.stamp;
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
    alive_clock_.roscco_can = msg->header.stamp;
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
void AutowareInterface::AWmodecallback(const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg)
{
    aw_current_mode_ = msg->mode;
}
void AutowareInterface::TCclockCallback(const rosgraph_msgs::msg::Clock clock_msg)
{
    alive_clock_.tc = clock_msg.clock;
}
void AutowareInterface::ROSCCOclockCallback(const rosgraph_msgs::msg::Clock clock_msg)
{
    alive_clock_.roscco = clock_msg.clock;
}
void AutowareInterface::OusterclockCallback(const rosgraph_msgs::msg::Clock clock_msg)
{
    alive_clock_.os = clock_msg.clock;
}
void AutowareInterface::ADMAclockCallback(const adma_ros_driver_msgs::msg::AdmaDataScaled adma_msg)
{
    alive_clock_.adma = adma_msg.header.stamp;
}

void AutowareInterface::TimerCallback()
{    
    // To Autoware
    autoware_auto_vehicle_msgs::msg::VelocityReport AW_velocity_status_msg;
    autoware_auto_vehicle_msgs::msg::SteeringReport AW_steering_tire_status_msg;
    autoware_auto_vehicle_msgs::msg::ControlModeReport autoware_control_msg;

    AW_velocity_status_msg.header.stamp = this->now();
    AW_velocity_status_msg.header.frame_id = "base_link";
    AW_velocity_status_msg.longitudinal_velocity = velocity_ * KPH2MPS;
    AW_velocity_status_msg.heading_rate = ((velocity_ * KPH2MPS) * std::tan(steering_angle_ / 15.7)) / SOUL_WHEEL_BASE;
    AW_steering_tire_status_msg.stamp = this->now();
    AW_steering_tire_status_msg.steering_tire_angle = (steering_angle_ / 15.7);

    AW_velocity_status_pub_->publish(AW_velocity_status_msg);
    AW_steer_status_pub_->publish(AW_steering_tire_status_msg);

    autoware_control_msg.mode = 1;
    autoware_control_pub_->publish(autoware_control_msg);

    // To Roscco
    roscco_msgs::msg::ThrottleCommand ROSCCO_throttle_msg;
    roscco_msgs::msg::BrakeCommand ROSCCO_brake_msg;
    roscco_msgs::msg::SteeringCommand ROSCCO_steering_msg;

    const double dt = (this->now() - alive_clock_.tc).seconds();
    if(std::fabs(dt) > 0.1f)
    {
        ROSCCO_throttle_msg.throttle_position = 0.0;
        ROSCCO_brake_msg.brake_position = 0.0;
        ROSCCO_steering_msg.steering_torque = 0.0;
    }
    else
    {
        ROSCCO_throttle_msg.throttle_position = TC_throttle_cmd_;
        ROSCCO_brake_msg.brake_position = TC_brake_cmd_;
        ROSCCO_steering_msg.steering_torque = TC_steer_cmd_;
    }

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
    if(aw_current_mode_ == 1)
    {
        TC_steer_command_msg.data = 0.0;
    }

    TC_steer_status_msg.data = steering_angle_;

    if(roscco_status_.brake_enabled + roscco_status_.steer_enabled + roscco_status_.throttle_enabled == 3)
    {
        TC_roscco_status_msg.data = true;
    }
    else
    {
        if(aw_current_mode_ == 2)
        {
            std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request> request = 
                std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
            std::shared_future<std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Response>> result = 
                AW_stop_client_->async_send_request(request);
        }

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

    autoware_system_msgs::msg::ComponentStatus component_status_msg = IsComponentAlive(alive_clock_);
    component_status_pub_->publish(component_status_msg);



    bool is_steer_aligned = false;

    if(std::abs(steering_angle_) < 0.3)
    {
        is_steer_aligned = true;
    }
    else
    {
        is_steer_aligned = false;
    }

    std_msgs::msg::Bool steer_aligned_status_msg;
    steer_aligned_status_msg.data = is_steer_aligned;
    steer_aligned_status_pub_->publish(steer_aligned_status_msg);
}

inline autoware_system_msgs::msg::ComponentStatus AutowareInterface::IsComponentAlive(const AutowareInterface::AliveClock alive_clock)
{
    autoware_system_msgs::msg::ComponentStatus component_status_msg;
    component_status_msg.is_roscco_alive = (this->now() - alive_clock.roscco).seconds() > 0.5f ? false : true;
    component_status_msg.is_adma_alive = (this->now() - alive_clock.adma).seconds() > 0.5f ? false : true;
    component_status_msg.is_os_alive = (this->now() - alive_clock.os).seconds() > 0.5f ? false : true;
    component_status_msg.is_tc_alive = (this->now() - alive_clock.tc).seconds() > 0.5f ? false : true;
    component_status_msg.is_roscco_can_alive = (this->now() - alive_clock.roscco_can).seconds() > 0.5f ? false : true;
    component_status_msg.is_vehicle_can_alive = (this->now() - alive_clock.vehicle_can).seconds() > 0.5f ? false : true;
    return component_status_msg;
}

void AutowareInterface::ClockTimerCallback()
{
    rosgraph_msgs::msg::Clock clock_msg; //HJK_250311_A
    clock_msg.clock = now(); //HJK_250311_A
    clock_pub->publish(clock_msg); //HJK_250311_A
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutowareInterface>());
    rclcpp::shutdown();
    return 0;
}
