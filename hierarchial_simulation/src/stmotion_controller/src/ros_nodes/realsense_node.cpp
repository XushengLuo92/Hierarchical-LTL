#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "ros/ros.h"
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
using namespace std::chrono;
using namespace cv;


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "realsense_node");
        ros::NodeHandle nh("~");
        ROS_INFO_STREAM("namespace of nh = " << nh.getNamespace());
        rs2::pipeline p;
        p.start();
        ros::Rate loop_rate(30);
        
        rs2::frameset frames = p.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        float img_w = color_frame.as<rs2::video_frame>().get_width();
        float img_h = color_frame.as<rs2::video_frame>().get_height();
        
        image_transport::ImageTransport it(nh);
        image_transport::Publisher img_pub = it.advertise("realsense/color_image", 1);
        Mat color_img(Size(img_w, img_h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", color_img).toImageMsg();
        
        while(ros::ok)
        {
            frames = p.wait_for_frames();
            color_frame = frames.get_color_frame();

            Mat color_img(Size(img_w, img_h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
            msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", color_img).toImageMsg();
            img_pub.publish(msg);
            ros::spinOnce();
        }
        ROS_INFO_STREAM("Realsense exit!");
        ros::shutdown();
        return 0;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}



