#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "rcl_interfaces/msg/parameter_event.hpp"

#include <opencv2/opencv.hpp>

#include "sky360lib/api/camera/qhy_camera.hpp"
#include "sky360lib/api/utils/autoExposureControl.hpp"


class ImagePublisher 
    : public rclcpp::Node
{
public:
    ImagePublisher() 
        : Node("all_sky_image_publisher_node")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("sky360/camera/original", 10);
        
        parameter_subscriber_ = this->create_subscription<rcl_interfaces::msg::ParameterEvent>(
            "/parameter_events", 10, std::bind(&ImagePublisher::parameter_callback, this, std::placeholders::_1));
        
        qhy_camera_.set_debug_info(false);
        qhy_camera_.open("");
        qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Exposure, 20000.0);
        qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Gain, 5.0);
    }

    void start_publishing()
    {
        while(rclcpp::ok()) // checking for shutdown
        {
            cv::Mat image;
            qhy_camera_.get_frame(image, true);

            const double exposure = (double)qhy_camera_.get_camera_params().exposure;
            const double gain = (double)qhy_camera_.get_camera_params().gain;
            auto exposure_gain = auto_exposure_control_.calculate_exposure_gain(image, exposure, gain);
            qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Exposure, exposure_gain.exposure);
            qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Gain, exposure_gain.gain);

            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "test_camera";
            auto image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
            this->publisher_->publish(*image_msg);

            // Handle callbacks
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

    void parameter_callback(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
    {
        // Here you should process changes in parameters.
        // This is a skeleton, you'll need to add appropriate logic here.
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_subscriber_;
    sky360lib::camera::QhyCamera qhy_camera_;
    sky360lib::utils::AutoExposureControl auto_exposure_control_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto image_publisher = std::make_shared<ImagePublisher>();
    image_publisher->start_publishing();
    rclcpp::shutdown();
    return 0;
}
