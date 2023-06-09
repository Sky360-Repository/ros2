#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "rcl_interfaces/msg/parameter_event.hpp"

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher() 
        : Node("all_sky_image_publisher_node")
    {
        // this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image/image_raw", 10);
        this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image", 10);
        
        this->parameter_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10, std::bind(&ImagePublisher::parameter_callback, this, std::placeholders::_1));
        this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ImagePublisher::timer_callback, this));
    }

    void timer_callback()
    {
        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3); // Create a dummy image for example

        // Create a red circle
        cv::Point center(image.cols / 2, image.rows / 2);  // Center of the circle
        int radius = std::min(image.rows, image.cols) / 4;  // Radius of the circle
        cv::Scalar color(0, 0, 255);  // Red color in BGR
        int thickness = -1;  // Negative thickness means the circle is filled
        cv::circle(image, center, radius, color, thickness);

        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "test_camera";
        auto image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        this->publisher_->publish(*image_msg);
    }

    void parameter_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
        // Here you should process changes in parameters.
        // This is a skeleton, you'll need to add appropriate logic here.
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
