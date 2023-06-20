#include <chrono>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include "sky360_camera/msg/image_info.hpp"
#include "sky360_camera/msg/camera_info.hpp"
#include "sky360_camera/msg/bayer_image.hpp"

#include <sky360lib/api/camera/qhy_camera.hpp>
#include <sky360lib/api/utils/autoExposureControl.hpp>

#include "../../sky360_shared/include/parameter_node.hpp"

class AllSkyPublisher
    : public ParameterNode // rclcpp::Node
{
public:
    AllSkyPublisher()
        : ParameterNode("all_sky_image_publisher_node")
    {
        image_publisher_ = create_publisher<sky360_camera::msg::BayerImage>("sky360/camera/all_sky/bayer", 10);
        image_info_publisher_ = create_publisher<sky360_camera::msg::ImageInfo>("sky360/camera/all_sky/image_info", 10);
        camera_info_publisher_ = create_publisher<sky360_camera::msg::CameraInfo>("sky360/camera/all_sky/camera_info", 10);

        declare_parameters();
    }

    void start_publishing()
    {
        open_camera();

        auto camera_info = qhy_camera_.get_camera_info();
        auto camera_params = qhy_camera_.get_camera_params();
        create_camera_info_msg(camera_params, camera_info);

        cv::Mat image;
        double duration_total = 0.0;
        double frames = 0.0;
        while (rclcpp::ok())
        {
            auto start = std::chrono::high_resolution_clock::now();

            camera_params = qhy_camera_.get_camera_params();

            qhy_camera_.get_frame(image, false);

            apply_auto_exposure(image, camera_params);

            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = boost::uuids::to_string(uuid_generator_());

            auto image_info_msg = generate_image_info(header, camera_params, camera_info);

            publish_image(image, header, image_info_msg);

            image_info_publisher_->publish(image_info_msg);
            publish_camera_info(header);

            auto end = std::chrono::high_resolution_clock::now();
            duration_total += std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1.0e9;
            ++frames;
            if (duration_total > 3.0)
            {
                RCLCPP_INFO(get_logger(), "%f fps", frames / duration_total);
                duration_total = 0.0;
                frames = 0.0;
            }

            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

protected:
    std::vector<rcl_interfaces::msg::SetParametersResult> set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters_to_set) override
    {
        auto set_results = this->set_parameters(parameters_to_set);

        for (const auto &result : set_results)
        {
            if (!result.successful)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
            }
        }

        return set_results;
    }

private:
    rclcpp::Publisher<sky360_camera::msg::BayerImage>::SharedPtr image_publisher_;
    rclcpp::Publisher<sky360_camera::msg::ImageInfo>::SharedPtr image_info_publisher_;
    rclcpp::Publisher<sky360_camera::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    sky360lib::camera::QhyCamera qhy_camera_;
    sky360lib::utils::AutoExposureControl auto_exposure_control_;
    sky360lib::camera::QhyCamera::CameraInfo *camera_info;
    sky360_camera::msg::CameraInfo camera_info_msg_;
    boost::uuids::random_generator uuid_generator_;

    void declare_parameters()
    {
        std::vector<rclcpp::Parameter> parameters = {
            rclcpp::Parameter("gain", 5),
            rclcpp::Parameter("exposure", 20000)
        };
    }

    inline void open_camera()
    {
        qhy_camera_.set_debug_info(false);
        qhy_camera_.open("");
        qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Exposure, 20000.0);
        qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Gain, 5.0);

        // uint32_t x = ((uint32_t)qhy_camera_.get_camera_info()->chip.max_image_width - (uint32_t)qhy_camera_.get_camera_info()->chip.max_image_height) / 2;
        // uint32_t y = 0;
        // uint32_t width = qhy_camera_.get_camera_info()->chip.max_image_height;
        // uint32_t height = qhy_camera_.get_camera_info()->chip.max_image_height;
        // qhy_camera_.set_resolution(x, y, width, height);
    }

    inline void apply_auto_exposure(const cv::Mat &image, sky360lib::camera::QhyCamera::CameraParams &camera_params)
    {
        const double exposure = (double)camera_params.exposure;
        const double gain = (double)camera_params.gain;
        auto exposure_gain = auto_exposure_control_.calculate_exposure_gain(image, exposure, gain);
        qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Exposure, exposure_gain.exposure);
        qhy_camera_.set_control(sky360lib::camera::QhyCamera::ControlParam::Gain, exposure_gain.gain);
    }

    inline void publish_image(const cv::Mat &image, const std_msgs::msg::Header &header, const sky360_camera::msg::ImageInfo &image_info)
    {
        auto image_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, image).toImageMsg();

        sky360_camera::msg::BayerImage bayer_image_msg;
        bayer_image_msg.header = header;
        bayer_image_msg.image = *image_msg;
        bayer_image_msg.info = image_info;

        image_publisher_->publish(bayer_image_msg);
    }

    inline sky360_camera::msg::ImageInfo generate_image_info(std_msgs::msg::Header &header, const sky360lib::camera::QhyCamera::CameraParams &camera_params, const sky360lib::camera::QhyCamera::CameraInfo *camera_info)
    {
        sky360_camera::msg::ImageInfo image_info_msg;
        image_info_msg.header = header;

        image_info_msg.roi.start_x = camera_params.roi.start_x;
        image_info_msg.roi.start_y = camera_params.roi.start_y;
        image_info_msg.roi.width = camera_params.roi.width;
        image_info_msg.roi.height = camera_params.roi.height;
        image_info_msg.bpp = camera_params.bpp;
        image_info_msg.bayer_format = camera_info->bayer_format;
        image_info_msg.exposure = camera_params.exposure;
        image_info_msg.gain = camera_params.gain;
        image_info_msg.offset = camera_params.offset;
        image_info_msg.white_balance.r = camera_params.red_white_balance;
        image_info_msg.white_balance.g = camera_params.green_white_balance;
        image_info_msg.white_balance.b = camera_params.blue_white_balance;
        image_info_msg.contrast = camera_params.contrast;
        image_info_msg.brightness = camera_params.brightness;
        image_info_msg.gamma = camera_params.gamma;
        image_info_msg.channels = camera_params.channels;
        image_info_msg.bin_mode = (uint32_t)camera_params.bin_mode;
        image_info_msg.current_temp = qhy_camera_.get_current_temp();

        //image_info_publisher_->publish(image_info_msg);
        return image_info_msg;
    }

    inline void create_camera_info_msg(const sky360lib::camera::QhyCamera::CameraParams &camera_params, const sky360lib::camera::QhyCamera::CameraInfo *camera_info)
    {
        camera_info_msg_.id = camera_info->id;
        camera_info_msg_.model = camera_info->model;
        camera_info_msg_.serial_num = camera_info->serial_num;
        camera_info_msg_.overscan.start_x = camera_info->overscan.start_x;
        camera_info_msg_.overscan.start_y = camera_info->overscan.start_y;
        camera_info_msg_.overscan.width = camera_info->overscan.width;
        camera_info_msg_.overscan.height = camera_info->overscan.height;
        camera_info_msg_.effective.start_x = camera_info->effective.start_x;
        camera_info_msg_.effective.start_y = camera_info->effective.start_y;
        camera_info_msg_.effective.width = camera_info->effective.width;
        camera_info_msg_.effective.height = camera_info->effective.height;
        camera_info_msg_.chip.width_mm = camera_info->chip.width_mm;
        camera_info_msg_.chip.height_mm = camera_info->chip.height_mm;
        camera_info_msg_.chip.pixel_width_um = camera_info->chip.pixel_width_um;
        camera_info_msg_.chip.pixel_height_um = camera_info->chip.pixel_height_um;
        camera_info_msg_.chip.max_image_width = camera_info->chip.max_image_width;
        camera_info_msg_.chip.max_image_height = camera_info->chip.max_image_height;
        camera_info_msg_.chip.max_bpp = camera_info->chip.max_bpp;
        camera_info_msg_.bayer_format = camera_info->bayer_format;
        camera_info_msg_.is_color = camera_info->is_color;
        camera_info_msg_.is_cool = camera_info->is_cool;
        camera_info_msg_.has_bin1x1_mode = camera_info->has_bin1x1_mode;
        camera_info_msg_.has_bin2x2_mode = camera_info->has_bin2x2_mode;
        camera_info_msg_.has_bin3x3_mode = camera_info->has_bin3x3_mode;
        camera_info_msg_.has_bin4x4_mode = camera_info->has_bin4x4_mode;
        camera_info_msg_.gain_limits.min = camera_info->gain_limits.min;
        camera_info_msg_.gain_limits.max = camera_info->gain_limits.max;
        camera_info_msg_.gain_limits.step = camera_info->gain_limits.step;
        camera_info_msg_.offset_limits.min = camera_info->offset_limits.min;
        camera_info_msg_.offset_limits.max = camera_info->offset_limits.max;
        camera_info_msg_.offset_limits.step = camera_info->offset_limits.step;
        camera_info_msg_.usb_traffic_limits.min = camera_info->usb_traffic_limits.min;
        camera_info_msg_.usb_traffic_limits.max = camera_info->usb_traffic_limits.max;
        camera_info_msg_.usb_traffic_limits.step = camera_info->usb_traffic_limits.step;
        camera_info_msg_.red_wb_limits.min = camera_info->red_wb_limits.min;
        camera_info_msg_.red_wb_limits.max = camera_info->red_wb_limits.max;
        camera_info_msg_.red_wb_limits.step = camera_info->red_wb_limits.step;
        camera_info_msg_.green_wb_limits.min = camera_info->green_wb_limits.min;
        camera_info_msg_.green_wb_limits.max = camera_info->green_wb_limits.max;
        camera_info_msg_.green_wb_limits.step = camera_info->green_wb_limits.step;
        camera_info_msg_.blue_wb_limits.min = camera_info->blue_wb_limits.min;
        camera_info_msg_.blue_wb_limits.max = camera_info->blue_wb_limits.max;
        camera_info_msg_.blue_wb_limits.step = camera_info->blue_wb_limits.step;
        camera_info_msg_.temperature_limits.min = camera_info->temperature_limits.min;
        camera_info_msg_.temperature_limits.max = camera_info->temperature_limits.max;
        camera_info_msg_.temperature_limits.step = camera_info->temperature_limits.step;
        camera_info_msg_.cool_enabled = camera_params.cool_enabled;
        camera_info_msg_.target_temp = camera_params.target_temp;
    }

    inline void publish_camera_info(std_msgs::msg::Header &header)
    {
        camera_info_msg_.header = header;

        camera_info_publisher_->publish(camera_info_msg_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto image_publisher = std::make_shared<AllSkyPublisher>();
    image_publisher->start_publishing();
    rclcpp::shutdown();
    return 0;
}
