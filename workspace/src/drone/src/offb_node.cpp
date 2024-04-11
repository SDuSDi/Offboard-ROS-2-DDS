// ROS libraries and dependencies
#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"

//JSON library
#include <nlohmann/json.hpp>
using json = nlohmann::json; // for convenience
// Callback file
#include "mqtt.cpp"

class OffboardControl : public rclcpp::Node
{
public:

    OffboardControl() : Node("offboard_control")
    {
        //vehicle_commander = this -> create_publisher<VehicleCommand>("/fmu/in/vehicle_command",10);
    }
    // void arm();
    // void disarm();

private:

    // rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_commander;
    
    // void OffboardControl::arm(){

    // }

};

int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<OffboardControl>());

    rclcpp::shutdown();
    return 0;
}