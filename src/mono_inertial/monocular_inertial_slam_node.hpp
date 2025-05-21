#ifndef ORBSLAM3_ROS2_ORBSLAM3_NODE_HPP
#define ORBSLAM3_ROS2_ORBSLAM3_NODE_HPP

#include <deque>
#include <mutex>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <System.h>
#include "ImuTypes.h"

namespace orbslam3_ros2 {

class ORBSLAM3Node : public rclcpp::Node
{
public:
    ORBSLAM3Node(const std::string &strVocFile, 
                const std::string &strSettingsFile, 
                const std::string &file_name = "");
    ~ORBSLAM3Node();

private:
    struct ImuData {
        double t;
        cv::Point3f acc;
        cv::Point3f gyr;
        
        ImuData(double _t, cv::Point3f _acc, cv::Point3f _gyr) 
            : t(_t), acc(_acc), gyr(_gyr) {}
    };

    // Callbacks
    void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg);
    void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);
    void ProcessFrame(const cv::Mat& im, double tframe);

    // SLAM system
    ORB_SLAM3::System* mpSLAM;
    float mImageScale;
    cv::Ptr<cv::CLAHE> mClahe;
    
    // ROS2 subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr mImuSub;
    
    // Data buffers and synchronization
    std::deque<ImuData> mImuBuffer;
    std::mutex mMutexImu;
    std::condition_variable mConVarImu;
    double mLastImageTime = 0.0;
    bool mFirstImage = true;
};

} // namespace orbslam3_ros2

#endif
