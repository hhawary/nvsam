#include <iostream>
#include "nvsam/libnvsam.hpp"
#include <rclcpp/rclcpp.hpp>

#include "zed_components/zed_camera_component.hpp"

int main(int argc, char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    // Option 1
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    rclcpp::NodeOptions options;

    options.use_intra_process_comms(true);

    auto nvsam_node = create_module(exec, options);
    if (!nvsam_node) {
        std::cout << "Not success!" << std::endl;
        return EXIT_FAILURE;
    }

    auto zed_node = std::make_shared<stereolabs::ZedCamera>(options);
    exec.add_node(zed_node);
    exec.spin();
    
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
