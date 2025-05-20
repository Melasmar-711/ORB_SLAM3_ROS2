#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "monocular_inertial_slam_node.hpp"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << std::endl << "Usage: ./ros2_mono_inertial path_to_vocabulary path_to_settings (trajectory_file_name)" << std::endl;
        return 1;
    }

    std::string file_name;
    if(argc == 4)
    {
        file_name = std::string(argv[3]);
    }

    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<orbslam3_ros2::ORBSLAM3Node>(argv[1], argv[2], file_name);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
