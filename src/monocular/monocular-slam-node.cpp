#include "monocular-slam-node.hpp"
#include <chrono>

using namespace std::chrono_literals;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM, const rclcpp::NodeOptions& options, bool use_webcam, int webcam_index)
: Node("ORB_SLAM3_ROS2", options), 
  m_SLAM(pSLAM),
  m_use_webcam(use_webcam),
  m_webcam_index(webcam_index)
{
    if (!m_use_webcam) {
        m_image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "/sf/AUV/rgb_camera/image_color",
            10,
            std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    } else {
m_cap.open(m_webcam_index);
	if(!m_cap.isOpened()) {
	    RCLCPP_FATAL(this->get_logger(), "Failed to open webcam %d", m_webcam_index);
	    rclcpp::shutdown();
	    return;
	}

	// Verify actual resolution
	int actual_width = m_cap.get(cv::CAP_PROP_FRAME_WIDTH);
	int actual_height = m_cap.get(cv::CAP_PROP_FRAME_HEIGHT);
	RCLCPP_INFO(this->get_logger(), "Webcam opened at %dx%d", actual_width, actual_height);
    }
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Shutdown SLAM first
    if(m_SLAM) {
        m_SLAM->Shutdown();
    }
    
    // Then release webcam
    if(m_use_webcam && m_cap.isOpened()) {
        m_cap.release();
    }
    
    // Clear OpenCV windows
    cv::destroyAllWindows();
}

void MonocularSlamNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
        ProcessFrame(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void MonocularSlamNode::RunFromCamera()
{
    cv::Mat frame;
    while (rclcpp::ok()) {
        m_cap >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame received from webcam");
            continue;
        }
        
        auto now = this->now();
        ProcessFrame(frame, now.seconds());
        cv::waitKey(1);
    }
}
void MonocularSlamNode::ProcessFrame(const cv::Mat& frame, double timestamp)
{
    cv::Mat gray, equalized;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Contrast enhancement
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8,8));
    clahe->apply(gray, equalized);
    
    // Show diagnostic windows
    cv::imshow("Webcam Input", equalized);
    
    // Track with enhanced image
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(equalized, timestamp);
    
    if(Tcw.log().norm() > 0) {
        RCLCPP_INFO(this->get_logger(), "Tracking OK!");
    } else {
        RCLCPP_INFO(this->get_logger(), 
                   "Move camera sideways to initialize (parallel to scene)");
    }
    cv::waitKey(1);
}
