#include "monocular_inertial_slam_node.hpp"

namespace orbslam3_ros2 {

ORBSLAM3Node::ORBSLAM3Node(const std::string &strVocFile, 
                          const std::string &strSettingsFile, 
                          const std::string &file_name) 
    : Node("orbslam3")
{
    // Create SLAM system
    mpSLAM = new ORB_SLAM3::System(strVocFile, strSettingsFile, 
                                  ORB_SLAM3::System::IMU_MONOCULAR, true, 0, file_name);
    mImageScale = mpSLAM->GetImageScale();
    mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

    // Initialize subscribers
    mImageSub = this->create_subscription<sensor_msgs::msg::Image>(
        "/sf/AUV/rgb_camera/image_color",
        rclcpp::SensorDataQoS(),
        std::bind(&ORBSLAM3Node::ImageCallback, this, std::placeholders::_1));

    mImuSub = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data",
        rclcpp::SensorDataQoS(),
        std::bind(&ORBSLAM3Node::ImuCallback, this, std::placeholders::_1));

    // Create OpenCV window
    cv::namedWindow("ORB-SLAM3: Tracking", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "ORB-SLAM3 ROS2 node initialized");
}

ORBSLAM3Node::~ORBSLAM3Node()
{
    // Stop all threads
    mpSLAM->Shutdown();
    
    // Save trajectory
    mpSLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
    mpSLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    
    // Close OpenCV window
    cv::destroyWindow("ORB-SLAM3: Tracking");
    
    delete mpSLAM;
}

void ORBSLAM3Node::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg)
{
    std::lock_guard<std::mutex> lock(mMutexImu);
    
    double imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
    cv::Point3f acc(imu_msg->linear_acceleration.x, 
                   imu_msg->linear_acceleration.y, 
                   imu_msg->linear_acceleration.z);
    cv::Point3f gyr(imu_msg->angular_velocity.x, 
                   imu_msg->angular_velocity.y, 
                   imu_msg->angular_velocity.z);
    
    mImuBuffer.push_back(ImuData(imu_time, acc, gyr));
    mConVarImu.notify_one();
}

void ORBSLAM3Node::ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg)
{
    try {
        // Process image
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);

        // Apply CLAHE
        cv::Mat im;
        mClahe->apply(cv_ptr->image, im);

        // Display the frame
        cv::imshow("ORB-SLAM3: Tracking", im);
        cv::waitKey(1);

        // Process frame with IMU data
        double tframe = rclcpp::Time(img_msg->header.stamp).seconds();
        ProcessFrame(im, tframe);
    }
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void ORBSLAM3Node::ProcessFrame(const cv::Mat& im, double tframe)
{
    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    
    // Wait for IMU data if this is the first frame
    if(mFirstImage) {
        std::unique_lock<std::mutex> lock(mMutexImu);
        mConVarImu.wait(lock, [this]{return !mImuBuffer.empty();});
        mFirstImage = false;
    }

    // Get IMU measurements since last image
    {
        std::lock_guard<std::mutex> lock(mMutexImu);
        
        // Remove old IMU measurements
        while(!mImuBuffer.empty() && mImuBuffer.front().t <= mLastImageTime) {
            mImuBuffer.pop_front();
        }

        // Get measurements up to current frame
        for(const auto& imu_data : mImuBuffer) {
            if(imu_data.t <= tframe) {
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                    imu_data.acc.x, imu_data.acc.y, imu_data.acc.z,
                    imu_data.gyr.x, imu_data.gyr.y, imu_data.gyr.z,
                    imu_data.t));
            } else {
                // Include one measurement beyond for interpolation
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(
                    imu_data.acc.x, imu_data.acc.y, imu_data.acc.z,
                    imu_data.gyr.x, imu_data.gyr.y, imu_data.gyr.z,
                    imu_data.t));
                break;
            }
        }
    }

    // Track only if we have IMU data
    if(!vImuMeas.empty()) {
        mpSLAM->TrackMonocular(im, tframe, vImuMeas);
    } else {
        RCLCPP_WARN(this->get_logger(), "Skipping frame - no IMU data available");
    }

    mLastImageTime = tframe;
}

} // namespace orbslam3_ros2
