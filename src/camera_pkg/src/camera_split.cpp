#include <camera_pkg/camera_split.hpp>

namespace camera_split_ns
{
    Camera_Split::Camera_Split():Node("image_cali_node"),it_(std::shared_ptr<rclcpp::Node>(std::move(this)))
    {
        RCLCPP_INFO(get_logger(),"Image_cali_node is running.");
        img_raw_sub=this->create_subscription<sensor_msgs::msg::Image>("/usb_cam/image_raw", 1, std::bind(&Camera_Split::img_raw_callback, this, _1));
        left_img_real_pub=it_.advertiseCamera("/left_cam/image_raw", 1);
        right_img_real_pub=it_.advertiseCamera("/right_cam/image_raw", 1);

        std::string left_cam_info,right_cam_info;
        this->declare_parameter("left_cam_info", "");
        this->declare_parameter("right_cam_info", "");
        this->get_parameter("left_cam_info",left_cam_info);
        this->get_parameter("right_cam_info",right_cam_info);

        if(!left_cam_info.empty())
        {
            if(cim_prt->validateURL(left_cam_info))
            {
                RCLCPP_INFO(get_logger(),"Load left_cam_info.");
                cim_prt->loadCameraInfo(left_cam_info);
                ci_left_ptr=sensor_msgs::msg::CameraInfo::SharedPtr(new sensor_msgs::msg::CameraInfo(cim_prt->getCameraInfo()));
            }
            else
            {
                RCLCPP_ERROR(get_logger(),"Failed to load left_cam_info.");
                rclcpp::shutdown();
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(),"Not found left_cam_info.");
            ci_left_ptr=sensor_msgs::msg::CameraInfo::SharedPtr(new sensor_msgs::msg::CameraInfo());
        }

        if(!right_cam_info.empty())
        {
            if(cim_prt->validateURL(right_cam_info))
            {
                RCLCPP_INFO(get_logger(),"Load right_cam_info.");
                cim_prt->loadCameraInfo(right_cam_info);
                ci_right_ptr=sensor_msgs::msg::CameraInfo::SharedPtr(new sensor_msgs::msg::CameraInfo(cim_prt->getCameraInfo()));
            }
            else
            {
                RCLCPP_ERROR(get_logger(),"Failed to load right_cam_info.");
                rclcpp::shutdown();
            }
        }
        else
        {
            RCLCPP_WARN(get_logger(),"Not found right_cam_info.");
            ci_right_ptr=sensor_msgs::msg::CameraInfo::SharedPtr(new sensor_msgs::msg::CameraInfo());
        }

    }

    Camera_Split::~Camera_Split()
    {}

    void Camera_Split::img_raw_callback(const sensor_msgs::msg::Image::SharedPtr img_raw_buf)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_ptr=cv_bridge::toCvShare(img_raw_buf,sensor_msgs::image_encodings::BGR8);

        cv::Mat mat_buf=cv_ptr->image(cv::Rect(0, 0, cv_ptr->image.cols/2, cv_ptr->image.rows));
        cv_left_ptr=cv_bridge::CvImagePtr(new cv_bridge::CvImage(cv_ptr->header,cv_ptr->encoding,mat_buf));
        mat_buf=cv_ptr->image(cv::Rect(cv_ptr->image.cols/2, 0, cv_ptr->image.cols/2, cv_ptr->image.rows));
        cv_right_ptr=cv_bridge::CvImagePtr(new cv_bridge::CvImage(cv_ptr->header,cv_ptr->encoding,mat_buf));

        ci_left_ptr->header=cv_ptr->header;
        ci_right_ptr->header=cv_ptr->header;
        sensor_msgs::msg::Image::SharedPtr img_left_ptr=cv_left_ptr->toImageMsg();
        sensor_msgs::msg::Image::SharedPtr img_right_ptr=cv_right_ptr->toImageMsg();
        img_left_ptr->header=img_raw_buf->header;
        img_right_ptr->header=img_raw_buf->header;
        left_img_real_pub.publish(img_left_ptr,ci_left_ptr);
        right_img_real_pub.publish(img_right_ptr,ci_right_ptr);
    }

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<camera_split_ns::Camera_Split>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
