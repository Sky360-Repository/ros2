#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include "sky360_camera/msg/image_info.hpp"
#include "sky360_camera/msg/camera_info.hpp"
#include "sky360_camera/msg/bayer_image.hpp"
#include "sky360_camera/msg/bayer_format.hpp"

class FrameProvider 
    : public rclcpp::Node
{
public:
    FrameProvider() 
        : Node("frame_provider_node")
    {
        image_subscription_ = create_subscription<sky360_camera::msg::BayerImage>("sky360/camera/all_sky/bayer", rclcpp::QoS(10),
            std::bind(&FrameProvider::imageCallback, this, std::placeholders::_1));

        gray_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/all_sky/gray", rclcpp::QoS(10));
        masked_publisher_ = create_publisher<sensor_msgs::msg::Image>("sky360/frames/all_sky/masked", rclcpp::QoS(10));
        image_info_publisher_ = create_publisher<sky360_camera::msg::ImageInfo>("sky360/frames/all_sky/image_info", rclcpp::QoS(10));
    }

private:
    void imageCallback(const sky360_camera::msg::BayerImage::SharedPtr msg)
    {
        try
        {
            auto start = std::chrono::high_resolution_clock::now();

            cv_bridge::CvImagePtr bayer_img_bridge = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::MONO8);

            cv::Mat debayered_img;
            debayer_image(bayer_img_bridge->image, debayered_img, msg->info.bayer_format);

            cv::Mat color_img;
            uint32_t frame_width = debayered_img.size().width;
            uint32_t frame_height = debayered_img.size().height;
            if (true) // Resize frame
            {
                double aspect_ratio = (double)bayer_img_bridge->image.size().width / (double)bayer_img_bridge->image.size().height;
                frame_height = 960;
                frame_width = ((uint32_t)(aspect_ratio * (double)frame_height)) & 0xFFFFFFFE;
                cv::resize(debayered_img, color_img, cv::Size(frame_width, frame_height));
            }
            else
            {
                color_img = debayered_img;
            }

            auto color_image_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, color_img).toImageMsg();
            masked_publisher_->publish(*color_image_msg);

            sky360_camera::msg::ImageInfo frame_info_msg = msg->info;
            frame_info_msg.roi.start_x = 0;
            frame_info_msg.roi.start_y = 0;
            frame_info_msg.roi.width = frame_width;
            frame_info_msg.roi.height = frame_height;
            image_info_publisher_->publish(frame_info_msg);

            cv::Mat gray_img;
            cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);
            auto gray_image_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, gray_img).toImageMsg();
            gray_publisher_->publish(*gray_image_msg);

            auto end = std::chrono::high_resolution_clock::now();
            duration_total += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1.0e9;
            ++frames;
            if (duration_total > 3.0)
            {
                RCLCPP_INFO(get_logger(), "%f fps", frames / duration_total);
                duration_total = 0.0;
                frames = 0.0;
            }
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        }
    }

    static inline int convert_bayer_pattern(uint32_t _bayerFormat)
    {
        switch (_bayerFormat)
        {
        case sky360_camera::msg::BayerFormat::BAYER_GB:
            return cv::COLOR_BayerGR2BGR; //!< equivalent to GBRG Bayer pattern
        case sky360_camera::msg::BayerFormat::BAYER_GR:
            return cv::COLOR_BayerGB2BGR; //!< equivalent to GRBG Bayer pattern
        case sky360_camera::msg::BayerFormat::BAYER_BG:
            return cv::COLOR_BayerRG2BGR; //!< equivalent to BGGR Bayer pattern
        case sky360_camera::msg::BayerFormat::BAYER_RG:
            return cv::COLOR_BayerBG2BGR; //!< equivalent to RGGB Bayer pattern
        }
        return cv::COLOR_BayerGR2BGR;
    }

    void debayer_image(const cv::Mat &_image_in, cv::Mat &_image_out, uint32_t _bayerFormat) const
    {
        cv::cvtColor(_image_in, _image_out, convert_bayer_pattern(_bayerFormat));
    }

    rclcpp::Subscription<sky360_camera::msg::BayerImage>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr gray_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr masked_publisher_;
    rclcpp::Publisher<sky360_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
    double duration_total = 0.0;
    double frames = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameProvider>());
    rclcpp::shutdown();
    return 0;
}
