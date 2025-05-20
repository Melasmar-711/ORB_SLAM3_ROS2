#include "monocular_inertial_slam_node.hpp"

namespace orbslam3_ros2 {

ORBSLAM3Node::ORBSLAM3Node(const std::string &strVocFile, 
                          const std::string &strSettingsFile, 
                          const std::string &file_name) 
    : Node("orbslam3"),
      mImageSub(this, "/sf/AUV/rgb_camera/image_color"),
      mImuSub(this, "/imu/data")
{
    // Create SLAM system
    mpSLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, 
                                  ORB_SLAM3::System::IMU_MONOCULAR, true, 0, file_name);
    mImageScale = mpSLAM->GetImageScale();

    // Initialize subscribers with QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).best_effort();
    
    // Synchronize image and IMU messages
    mSync = std::make_shared<message_filters::Synchronizer<sync_pol>>(
        sync_pol(10), mImageSub, mImuSub);
    mSync->registerCallback(&ORBSLAM3Node::GrabImuAndImage, this);

    mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    
    // Initialize IMU buffer
    mImuBuffer.clear();
    mLastImageTime = 0.0;
    
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS2 node initialized");
}

ORBSLAM3Node::~ORBSLAM3Node()
{
    // Stop all threads
    mpSLAM->Shutdown();
    
    // Save trajectory
    mpSLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    mpSLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    
    delete mpSLAM;
}

void ORBSLAM3Node::GrabImuAndImage(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg, 
                                  const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
    try {
        // Process IMU data first
        {
            std::lock_guard<std::mutex> lock(mMutexImu);
            
            double t = rclcpp::Time(img_msg->header.stamp).seconds();
            cv::Point3f acc(imu_msg->linear_acceleration.x, 
                           imu_msg->linear_acceleration.y, 
                           imu_msg->linear_acceleration.z);
            cv::Point3f gyr(imu_msg->angular_velocity.x, 
                            imu_msg->angular_velocity.y, 
                            imu_msg->angular_velocity.z);
            
            mImuBuffer.push_back(ImuData(t, acc, gyr));
        }

        // Process image
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);

        // Apply CLAHE
        cv::Mat im;
        mClahe->apply(cv_ptr->image, im);

        // Handle image scaling
        if(mImageScale != 1.f)
        {
            int width = im.cols * mImageScale;
            int height = im.rows * mImageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Get timestamp
        double tframe = rclcpp::Time(img_msg->header.stamp).seconds();

        // Get IMU measurements since last image
        std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
        GetImuMeasurements(tframe, vImuMeas);

        // Track the image with IMU data
        mpSLAM->TrackMonocular(im, tframe, vImuMeas);

        // Update last image time
        mLastImageTime = tframe;
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        return;
    }
}

void ORBSLAM3Node::GetImuMeasurements(double tframe, std::vector<ORB_SLAM3::IMU::Point>& vImuMeas)
{
    std::lock_guard<std::mutex> lock(mMutexImu);
    
    if(mImuBuffer.empty())
        return;
        
    // Remove IMU measurements older than last image
    while(!mImuBuffer.empty() && mImuBuffer.front().t <= mLastImageTime)
    {
        mImuBuffer.pop_front();
    }

    // Get IMU measurements between last image and current image
    for(const auto& imu_data : mImuBuffer)
    {
        if(imu_data.t <= tframe)
        {
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                imu_data.acc.x, imu_data.acc.y, imu_data.acc.z,
                imu_data.gyr.x, imu_data.gyr.y, imu_data.gyr.z,
                imu_data.t));
        }
        else
        {
            break;
        }
    }
}

} // namespace orbslam3_ros2
