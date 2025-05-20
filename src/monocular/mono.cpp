#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "monocular-slam-node.hpp"
#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 3) {
        std::cerr << "\nUsage: ros2 run orbslam mono path_to_vocabulary path_to_settings [--webcam [index]]\n";
        return 1;
    }

    rclcpp::init(argc, argv);

    bool use_webcam = false;
    int webcam_index = 0;
    
    for(int i = 0; i < argc; ++i) {
        if(std::string(argv[i]) == "--webcam") {
            use_webcam = true;
            if(i+1 < argc) webcam_index = atoi(argv[i+1]);
            break;
        }
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    auto node = std::make_shared<MonocularSlamNode>(&SLAM, rclcpp::NodeOptions(), use_webcam, webcam_index);

    if(use_webcam) {
        node->RunFromCamera();
    } else {
        rclcpp::spin(node);
    }

    rclcpp::shutdown();
    return 0;
}
