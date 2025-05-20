#ifndef ORBSLAM3_ROS2_ORBSLAM3_NODE_HPP
#define ORBSLAM3_ROS2_ORBSLAM3_NODE_HPP

#include <memory>
#include <deque>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

#include <System.h>
#include "ImuTypes.h"

namespace orbslam3_ros2 {

class ORBSLAM3Node : public rclcpp::Node
{
public:
    ORBSLAM3Node(const std::string &strVocFile, 
                const std::string &strSettingsFile, 
                const std::string &file_name);
    ~ORBSLAM3Node();

private:
    // Define synchronization policy
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, 
        sensor_msgs::msg::Imu> sync_pol;

    struct ImuData {
        double t;
        cv::Point3f acc;
        cv::Point3f gyr;
        
        ImuData(double _t, cv::Point3f _acc, cv::Point3f _gyr) 
            : t(_t), acc(_acc), gyr(_gyr) {}
    };

    void GrabImuAndImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, 
                        const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg);
    void GetImuMeasurements(double tframe, std::vector<ORB_SLAM3::IMU::Point>& vImuMeas);

    ORB_SLAM3::System* mpSLAM;
    float mImageScale;
    cv::Ptr<cv::CLAHE> mClahe;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> mImageSub;
    message_filters::Subscriber<sensor_msgs::msg::Imu> mImuSub;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> mSync;
    
    std::deque<ImuData> mImuBuffer;
    std::mutex mMutexImu;
    double mLastImageTime;
};

} // namespace orbslam3_ros2

#endif // ORBSLAM3_ROS2_ORBSLAM3_NODE_HPP
