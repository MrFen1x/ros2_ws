#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

class CameraPublisher : public rclcpp::Node
{
public:
  CameraPublisher()
      : Node("camera_publisher"), count_(0)
  {
    // Публикуем Image, а не String
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&CameraPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    cv::VideoCapture cam("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)20/1 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink", cv::CAP_GSTREAMER);
    
    if(!cam.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Camera is not opened.");
      return;
    }
    
    cv::Mat frame;
    bool success = cam.read(frame);
    
    if(!success || frame.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
      return;
    }
    
    // Конвертируем cv::Mat в sensor_msgs::Image using cv_bridge
    auto message = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    message->header.stamp = this->now();
    message->header.frame_id = "camera_frame";
    
    publisher_->publish(*message);
    RCLCPP_INFO(this->get_logger(), "Published image %zu", count_++);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}