#include "yolov8_wrapper/detect.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Detector>();

    // Run the BuggyOdom
    node->run();

    rclcpp::shutdown();
    return 0;
}

