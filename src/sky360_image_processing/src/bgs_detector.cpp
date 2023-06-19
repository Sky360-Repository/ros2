#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/bounding_box2_d_array.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <sky360lib/api/blobs/connectedBlobDetection.hpp>

class BackgroundSubtractorDetector
    : public rclcpp::Node
{
public:
    BackgroundSubtractorDetector()
        : Node("background_subtractor_detector_node")
    {
        image_subscription_ = create_subscription<sensor_msgs::msg::Image>("sky360/frames/foreground_mask", rclcpp::QoS(10),
            std::bind(&BackgroundSubtractorDetector::imageCallback, this, std::placeholders::_1));

        detection_publisher_ = create_publisher<vision_msgs::msg::BoundingBox2DArray>("sky360/detector/bgs/bounding_boxes", rclcpp::QoS(10));
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try
        {
            auto start = std::chrono::high_resolution_clock::now();
            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

            std::vector<cv::Rect> bboxes;
            if (blob_detector_.detect(cv_image->image, bboxes))
            {
                vision_msgs::msg::BoundingBox2DArray bbox2D_array;
                bbox2D_array.header = msg->header;
                convert_bboxes(bbox2D_array, bboxes);

                detection_publisher_->publish(bbox2D_array);
            }

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

    void convert_bboxes(vision_msgs::msg::BoundingBox2DArray& bbox2D_array, std::vector<cv::Rect>& bboxes)
    {
        for (const auto &bbox : bboxes)
        {
            vision_msgs::msg::BoundingBox2D bbox2D;
            bbox2D.center.position.x = bbox.x + bbox.width / 2.0;
            bbox2D.center.position.y = bbox.y + bbox.height / 2.0;
            bbox2D.size_x = bbox.width;
            bbox2D.size_y = bbox.height;
            bbox2D_array.boxes.push_back(bbox2D);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<vision_msgs::msg::BoundingBox2DArray>::SharedPtr detection_publisher_;
    sky360lib::blobs::ConnectedBlobDetection blob_detector_;
    double duration_total = 0.0;
    double frames = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BackgroundSubtractorDetector>());
    rclcpp::shutdown();
    return 0;
}
