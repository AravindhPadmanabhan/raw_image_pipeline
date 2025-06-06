#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CompressedImageConverter {
public:
    CompressedImageConverter(ros::NodeHandle& nh) {
        std::string input_topic, output_topic;
        nh.param<std::string>("input_topic", input_topic, "/alphasense_driver_ros/cam0/debayered/image/compressed");
        nh.param<std::string>("output_topic", output_topic, "/alphasense_driver_ros/cam0/debayered/image");

        sub_ = nh.subscribe(input_topic, 1, &CompressedImageConverter::imageCallback, this);
        pub_ = nh.advertise<sensor_msgs::Image>(output_topic, 1);

        ROS_INFO("Input topic: %s", input_topic.c_str());
        ROS_INFO("Output topic: %s", output_topic.c_str());

    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;

    void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
        try {
            // Decode compressed image using OpenCV
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (image.empty()) {
                ROS_ERROR("Failed to decode compressed image.");
                return;
            }

            ROS_INFO("Received compressed image with size: %d bytes", msg->data.size());

            // Convert OpenCV image to ROS message
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();

            // Publish the uncompressed image
            pub_.publish(img_msg);
        } catch (const cv::Exception& e) {
            ROS_ERROR("OpenCV error: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    CompressedImageConverter converter(nh);
    ros::spin();
    return 0;
}