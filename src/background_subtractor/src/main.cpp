#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "sky360lib/api/bgs/bgs.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class BackgroundSubtractor 
    : public rclcpp::Node
{
public:
    BackgroundSubtractor() 
        : Node("background_subtractor_node")
    {
        // Subscribe to the input image topic
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
            "sky360/camera/original", rclcpp::QoS(10),
            std::bind(&BackgroundSubtractor::imageCallback, this, std::placeholders::_1));

        // Publish the manipulated image
        image_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/foreground_mask", rclcpp::QoS(10));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            cv::Mat gray_img;
            cv::cvtColor(cv_image->image, gray_img, cv::COLOR_BGR2GRAY);

            cv::Mat mask;
            background_subtractor_.apply(gray_img, mask);
            
            auto image_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, mask).toImageMsg();
            image_publisher_->publish(*image_msg);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    sky360lib::bgs::WeightedMovingVariance background_subtractor_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackgroundSubtractor>());
    rclcpp::shutdown();
    return 0;
}
