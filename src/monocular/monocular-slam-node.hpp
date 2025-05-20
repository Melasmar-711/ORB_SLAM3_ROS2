#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "utility.hpp"

class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM, const rclcpp::NodeOptions& options = rclcpp::NodeOptions(), bool use_webcam = false, int webcam_index = 0);
    ~MonocularSlamNode();

    void RunFromCamera();

private:
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void ProcessFrame(const cv::Mat& frame, double timestamp);

    ORB_SLAM3::System* m_SLAM;
    cv_bridge::CvImagePtr m_cvImPtr;
    cv::VideoCapture m_cap;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    bool m_use_webcam;
    int m_webcam_index;
};

#endif
