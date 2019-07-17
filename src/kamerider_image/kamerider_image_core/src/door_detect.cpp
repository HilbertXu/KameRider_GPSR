/*
Date： 2018/12/12
Author: Xu Yucheng
使用PCL来检测机器人面前的门是否打开，检测当前画面中点云的数目
Astra PRO大概在离门34cm之内检测到点云数目为0
若门打开可以判断100帧内检测到的点云数目是否大于90000
*/
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/thread.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
//体素网格采样
#include <pcl_ros/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <kamerider_control_msgs/Result.h>

using namespace std;
using namespace cv;
using namespace pcl;

// pcl container
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_frame (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filter (new pcl::PointCloud<pcl::PointXYZI>);

class door_detect
{
private:
    // flags
    bool door_closed = false;
    bool door_opened = false;
    bool if_detect   = true;
    int step = 0;

    // subscriber & publisher
    ros::Publisher door_pub;
    ros::Subscriber image_sub;
    ros::Subscriber pcl_sub;

    // ros params
    std::string sub_image_raw_topic_name;
    std::string sub_pcl_topic_name;
    std::string pub_door_detect_topic_name;
    std::string path_to_save_image;
    
    void pclCallback (const sensor_msgs::PointCloud2& msg)
    {
        // 将ros消息类型的点云转化为pcl格式
        pcl::fromROSMsg (msg, *cloud_frame);
        // 剔除Nan点
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud (*cloud_frame, *cloud_frame, mapping);
        // 对点云进行体素滤波下采样
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud (*cloud_frame, *cloud_temp);
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setInputCloud (cloud_temp);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter(*cloud_filter);

        int point_num = cloud_filter->points.size();
        if (if_detect)
        {
            if (point_num <= 20000)
            {
                door_closed = true;
            }
            if (point_num >= 20000)
            {
                door_opened = true;
                step ++;
            }
        }
        
    }

    void imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
        // 使用cv_bridge将ros格式的图像信息转化为opencv格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr -> image;
        
        if (door_opened && step>=50)
        {
            ROS_INFO ("Now the door is opened");
            std::string FILE_NAME = "/door_opened.png";
            std::string full_path = door_detect::path_to_save_image + FILE_NAME;
            cv::imwrite (full_path, img);
            
            if_detect = false;
            //由发布器向外部发布门已经打开了的消息
            kamerider_control_msgs::Result msg;
            msg.mission_type = "door";
            msg.result = "open";
            door_detect::door_pub.publish (msg);
        }
    }

public:
    int run (int argc, char** argv)
    {
        // 初始化ROS节点
        ros::init (argc, argv, "door_detect");
        ros::NodeHandle nh;
        ROS_INFO ("----------INIT----------");
        ROS_INFO ("----Waiting for image----");

        // 从ROS param参数服务器中获取参数
        nh.param<std::string> ("sub_image_raw_topic_name"   , sub_image_raw_topic_name   , "/astra/rgb/image_raw");
        nh.param<std::string> ("sub_pcl_topic_name"         , sub_pcl_topic_name         , "/astra/depth/points");
        nh.param<std::string> ("pub_door_detect_topic_name" , pub_door_detect_topic_name , "/image_to_control");
        nh.param<std::string> ("path_to_save_image"         , path_to_save_image         , "/home/kamerider/catkin_ws/src/inspection");

        // 定义发布器和订阅器
        door_pub   = nh.advertise<kamerider_control_msgs::Result> (door_detect::pub_door_detect_topic_name, 1);
        image_sub  = nh.subscribe (door_detect::sub_image_raw_topic_name, 1, &door_detect::imageCallback, this);
        pcl_sub    = nh.subscribe (door_detect::sub_pcl_topic_name, 1, &door_detect::pclCallback, this);

        ros::spin();
    }
};

int main (int argc, char** argv)
{
    door_detect detector;
    return detector.run (argc, argv);
}
