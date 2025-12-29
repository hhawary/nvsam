#ifndef NVSAM_HPP
#define NVSAM_HPP

#include <rclcpp/rclcpp.hpp>

extern "C" {
    std::shared_ptr<rclcpp::Node> create_module(rclcpp::executors::MultiThreadedExecutor & exec, const rclcpp::NodeOptions & options);
}

#endif  // NVSAM_HPP

