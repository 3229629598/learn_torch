#include <camera_pkg/camera_split.hpp>

namespace camera_split_ns
{
    Camera_Split::Camera_Split():Node("camera_split_node")
    {
        RCLCPP_INFO(get_logger(),"Camera_split_node is running.");
        img_raw_sub=this->create_subscription<sensor_msgs::msg::Image>("/usb_cam/image_raw", 1, std::bind(&Camera_Split::img_raw_callback, this, std::placeholders::_1));
        left_img_raw_pub=this->create_publisher<sensor_msgs::msg::Image>("/left_cam/image_raw",1);
        right_img_raw_pub=this->create_publisher<sensor_msgs::msg::Image>("/right_cam/image_raw",1);
    }

    Camera_Split::~Camera_Split()
    {}

    void Camera_Split::img_raw_callback(const sensor_msgs::msg::Image::SharedPtr img_raw_buf)
    {
        try
        {
            cv_ptr=cv_bridge::toCvCopy(img_raw_buf, sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
            img_split(cv_ptr->image);
        }
        catch(cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(get_logger(),"cv_bridge exception: %s", e.what());
            return;
        } 
    }

    void Camera_Split::img_split(cv::Mat img)
    {
        cv::Mat split_mat=img(cv::Range(0,img.rows),cv::Range(0,img.cols/2));
        split_img_buf=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", split_mat).toImageMsg();
        left_img_raw_pub->publish(*split_img_buf);

        split_mat=img(cv::Range(0,img.rows),cv::Range(img.cols/2,img.cols));
        split_img_buf=cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", split_mat).toImageMsg();
        right_img_raw_pub->publish(*split_img_buf);
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
