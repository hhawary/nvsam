#include <iostream>
#include "nvsam/libnvsam.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("nvsam_node");
    create_module(node);
    rclcpp::shutdown();
    return 0;
}
