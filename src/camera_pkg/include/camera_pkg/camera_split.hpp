#ifndef camera_split_hpp
#define camera_split_hpp

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace camera_split_ns
{
    using namespace std::placeholders;

    class Camera_Split : public rclcpp::Node
    {
        public:
        Camera_Split();
        ~Camera_Split();
        private:
        image_transport::ImageTransport it_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_raw_sub;
        void img_raw_callback(const sensor_msgs::msg::Image::SharedPtr img_raw_buf);
        image_transport::CameraPublisher left_img_real_pub,right_img_real_pub;
        cv_bridge::CvImagePtr cv_left_ptr,cv_right_ptr;
        sensor_msgs::msg::CameraInfo::SharedPtr ci_left_ptr,ci_right_ptr;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cim_left_ptr,cim_right_ptr;
    };
}

#endif
