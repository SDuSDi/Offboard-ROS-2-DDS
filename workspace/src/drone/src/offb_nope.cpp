#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

rclcpp::Node::SharedPtr node;

int main(int argc, char **argv) {

    rclcpp::init(argc,argv);
    auto pub = node->create_publisher<std_msgs::msg::String>("publisher",10);
    int i = 0;
    while(rclcpp::ok()){
        std_msgs::msg::String msg;
        msg.data = "Hello World" + std::to_string(i);
        i++;
        pub->publish(msg);
    }

    return 0;
}