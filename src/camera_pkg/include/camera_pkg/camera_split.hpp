#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>

namespace camera_split_ns
{
    class Camera_Split : public rclcpp::Node
    {
        public:
        Camera_Split();
        ~Camera_Split();
        private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_raw_sub;
        void img_raw_callback(const sensor_msgs::msg::Image::SharedPtr img_raw_buf);
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_raw_pub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_raw_pub;
        cv_bridge::CvImagePtr cv_ptr;
        sensor_msgs::msg::Image::SharedPtr split_img_buf;
        void img_split(cv::Mat img);
    };
}
