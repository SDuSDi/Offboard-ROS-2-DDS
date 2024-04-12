// ROS libraries and dependencies
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

//JSON library
#include <nlohmann/json.hpp>
using json = nlohmann::json; // for convenience
// Callback file
#include "mqtt.cpp"

using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:

    OffboardControl() : Node("offb_node")
    {
        offboard_controller = this -> create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode",10);
        trajectory_publisher = this -> create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
        vehicle_commander = this -> create_publisher<VehicleCommand>("/fmu/in/vehicle_command",10);

        for (int i = 0; i < 10; i++){
            rclcpp::sleep_for(std::chrono::seconds(1));
            this -> publish_offboard_mode();
            this -> publish_trajectory(0,0,2.0);
        }

        this -> publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);
        this -> arm();
        this -> publish_offboard_mode();
        this -> publish_trajectory(0,0,2.0);

    }
    void arm();
    void disarm();

private:

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_controller;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_commander;

    void publish_offboard_mode();
    void publish_trajectory(float x = 0.0, float y = 0.0, float z = 2.0);
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    
};

void OffboardControl::arm(){
    this -> publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,1);
    RCLCPP_INFO(this -> get_logger(), "Vehicle is armed");
}

void OffboardControl::disarm(){
    this -> publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0);
    RCLCPP_INFO(this -> get_logger(), "Vehicle is disarmed");
}

void OffboardControl::publish_offboard_mode(){
    OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
	this -> offboard_controller -> publish(msg);
}

void OffboardControl::publish_trajectory(float x, float y, float z){
    TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = 0; // -3.14; // [-PI:PI]
	msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
	this -> trajectory_publisher -> publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2){
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
	this -> vehicle_commander -> publish(msg);  
}

int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}