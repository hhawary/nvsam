#ifndef NVSAM_HPP
#define NVSAM_HPP

#include <rclcpp/rclcpp.hpp>

extern "C" {
    bool create_module(std::shared_ptr<rclcpp::Node> node);
}

#endif  // NVSAM_HPP

