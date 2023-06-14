#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/parameter_event.hpp"
//#include "all_sky_image_publisher/msg/ImageWithInfo.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class FrameProvider 
    : public rclcpp::Node
{
public:
    FrameProvider() 
        : Node("frame_provider_node")
    {
        // Subscribe to the input image topic
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "sky360/camera/all_sky/bayered", rclcpp::QoS(10),
            std::bind(&FrameProvider::imageCallback, this, std::placeholders::_1));

        // Publish the manipulated image
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/original", rclcpp::QoS(10));
        grey_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/grey", rclcpp::QoS(10));
        masked_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/masked", rclcpp::QoS(10));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr bayer_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

            // cv::Mat gray_img;
            // cv::cvtColor(cv_image->image, gray_img, cv::COLOR_BGR2GRAY);

            // cv::Mat mask;
            // background_subtractor_.apply(gray_img, mask);
            
            // auto image_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg();
            
            // image_publisher_->publish(*image_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grey_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr masked_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameProvider>());
    rclcpp::shutdown();
    return 0;
}
