// ROS libraries and dependencies
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

#include <iostream>

// Callback files
#include "mqtt.cpp"
#include "pid.cpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

std::array<float,3> drone_pos = {0.0,0.0,0.0};
std::array<float,3> expected_pos = {0.01,0.01,2.0};

PID pidx(1.0,1.0,1.0,0.1);
PID pidy(1.0,1.0,1.0,0.1);
PID pidz(10.0,1.0,0.05,0.1);

class OffboardControl : public rclcpp::Node
{
public:

    OffboardControl() : Node("offb_node")
    {
        offboard_controller = this -> create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode",10);
        trajectory_publisher = this -> create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
        vehicle_commander = this -> create_publisher<VehicleCommand>("/fmu/in/vehicle_command",10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        vehicle_odometry = this -> create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", qos,
        [this] (const VehicleOdometry::SharedPtr msg) {
            drone_pos = msg -> position;
            // RCLCPP_INFO(this -> get_logger(), "Drone position: %f, %f, %f", drone_pos[0], drone_pos[1], drone_pos[2]);
        });

        set_pos = false;
		offboard_setpoint_counter_ = 0;

        if(set_pos){
            RCLCPP_INFO(this -> get_logger(), "Offboard control by position");
        }else{
            RCLCPP_INFO(this -> get_logger(), "Offboard control by velocity");
        }

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10)
            {
				this -> publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);
				this -> arm();
			}
            this -> publish_offboard_mode(set_pos, !set_pos);

            if(set_pos){
                this -> publish_trajectory(0, 0, 2, set_pos);
            }else{
                //std::cout<<pidx.calculate(expected_pos[0], drone_pos[0] + 1.0)<<std::endl;
                this -> publish_trajectory(
                    pidx.calculate(expected_pos[0], drone_pos[0] + 1.0),
                    pidy.calculate(expected_pos[1], drone_pos[1] + 1.0),
                    pidz.calculate(-expected_pos[2], drone_pos[2]),
                    set_pos
                );
                // this -> publish_trajectory(
                //     1 * (expected_pos[0] - (drone_pos[0] + 1.0)),
                //     1 * (expected_pos[1] - (drone_pos[1] + 1.0)),
                //     1 * (- expected_pos[2] - drone_pos[2]),
                //     set_pos
                // );
            }
			if (offboard_setpoint_counter_ < 11){offboard_setpoint_counter_++;}
		};
		timer = this -> create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();

private:

    rclcpp::TimerBase::SharedPtr timer;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_controller;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_commander;

    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry;

    bool set_pos;
    uint64_t offboard_setpoint_counter_;

    void publish_offboard_mode(bool check_pos = true, bool check_vel = false);
    void publish_trajectory(float x = 0.0, float y = 0.0, float z = 2.0, bool check = true);
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

void OffboardControl::publish_offboard_mode(bool check_pos, bool check_vel){
    OffboardControlMode msg{};
	msg.position = check_pos;
	msg.velocity = check_vel;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
	this -> offboard_controller -> publish(msg);
}

void OffboardControl::publish_trajectory(float x, float y, float z, bool check){
    if(check)
    {
        TrajectorySetpoint msg{};
        msg.position = {x, y, z};
        msg.yaw = 0; // -3.14; // [-PI:PI]
        msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
        this -> trajectory_publisher -> publish(msg);
    }
    else
    {
        TrajectorySetpoint msg{};
        msg.velocity = {x, y, z};
        msg.yaw = 0; // -3.14; // [-PI:PI]
        msg.timestamp = this -> get_clock() -> now().nanoseconds() / 1000;
        this -> trajectory_publisher -> publish(msg);
    }
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

// void OffboardControl::odometry_cb(const VehicleOdometry::SharedPtr msg){
//     drone_pos = msg -> position;
//     RCLCPP_INFO(this -> get_logger(), "Drone position: %f, %f, %f", drone_pos[0], drone_pos[1], drone_pos[2]);
// }

int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}