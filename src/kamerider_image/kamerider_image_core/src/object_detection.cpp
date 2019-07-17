/*
    Date: 2019/03/13
Author: Xu Yucheng 
    Abstract: object detection with yolo, 3D-position predict with PCL
*/
#define PI 3.1415926

// common headers
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <cmath>
#include <sstream>

// OpenCV headers
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>

// user-defined ROS message type
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <kamerider_image_msgs/ObjectPosition.h>
#include <kamerider_control_msgs/Mission.h>
#include <kamerider_control_msgs/Result.h>

using namespace std;
using namespace pcl;
using namespace cv;


pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_astra (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base (new pcl::PointCloud<pcl::PointXYZ>);
unsigned char floatBuffer[4];
    
bool find_object = false;
bool adjust_robot = false;
bool in_position = false;
bool pub_obj_position = false;
int count_detect = 0;
int camera_width = 640;
int camera_height = 480;
int object_x = 0;
int object_y = 0;
int origin_index = 0;
int valid_index = 0;

class object_detection
{
private:
    // ROS parameters
    std::string sub_bbox_topic_name;
    std::string sub_pcl_topic_name;
    std::string sub_image_raw_topic_name;
    std::string sub_control_topic_name;
    
    std::string pub_object_pos_topic_name;
    std::string pub_tf_pcl_topic_name;

    // ROS subscriber & publisher
    ros::Subscriber bbox_sub;
    ros::Subscriber pcl_sub;
    ros::Subscriber img_sub;
    ros::Subscriber control_sub;

    ros::Publisher obj_pub;
    ros::Publisher pcl_pub;

    // parameters
    std::string CAMERA_DEPTH_FRAME = "astra_depth_frame";
    std::string TURTLEBOT_BASE_FRAME = "base_link";

    // 定义待抓取的物体的名称，初始化为None
    std::string object_name = "cookies";
    
    int find_near_valid(int idx)
    {   
        double temp_min = 9999999999;
        int return_idx = idx;
        int obj_row = idx/camera_width;
        int obj_col = idx%camera_width;

        for (int row=0; row<camera_height; row++)
        {
            for (int col=0; col<camera_width; col++)
            {
                if (!isnan(cloud_astra->points[row*camera_width+col].x) &&
                    !isnan(cloud_astra->points[row*camera_width+col].y) &&
                    !isnan(cloud_astra->points[row*camera_width+col].z))
                {
                    double dis = (row-obj_row)*(row-obj_row) + (col-obj_col)*(col-obj_col);
                    if (dis < temp_min)
                    {
                        return_idx = row*camera_width + col;
                        temp_min = dis;
                    }
                }
            }
        }
        valid_index = return_idx;
        return return_idx;
    }

    void controlCallback(const kamerider_control_msgs::Mission msg)
    {
        if (msg.mission_type == "object")
        {
            object_name = msg.mission_name;
            in_position = true;
        }
    }

    void imageCallback (const sensor_msgs::ImageConstPtr& msg)
    {
        if (find_object && !cloud_astra->empty())
        {
            int ori_row = origin_index / camera_width;
            int ori_col = origin_index % camera_width;
            int val_row = valid_index / camera_width;
            int val_col = valid_index % camera_width;
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_ptr->image;
            Point ori(ori_col, ori_row);
            Point val(val_col, val_row);

            circle(img, ori, 3, Scalar(0,0,255), -1);
            circle(img, val, 3, Scalar(255,0,0), -1);

            cv::imshow("object_detection", img);
            cv::waitKey(10);
        }
    }
    
    void darknetCallback (darknet_ros_msgs::BoundingBoxes msg)
    {
        if (object_name != "none")
        {
            if (msg.bounding_boxes.size() > 0 && !cloud_astra->empty())
            {
                for (int i=0; i<msg.bounding_boxes.size(); i++)
                {
                    if (msg.bounding_boxes[i].Class == object_name)
                    {
                        object_x = int ((msg.bounding_boxes[i].xmin + msg.bounding_boxes[i].xmax) / 2);
                        object_y = int ((msg.bounding_boxes[i].ymin + msg.bounding_boxes[i].ymax) / 2);
                        // Used for test
                        // printf ("Detected %s Bounding_Boxes\n",object_name.c_str());
                        // printf ("Bounding_Boxes %s=[Xmin Ymin Xmax Ymax]=[%d %d %d %d]\n", object_name.c_str(),
                        // msg.bounding_boxes[i].xmin,
                        // msg.bounding_boxes[i].ymin,
                        // msg.bounding_boxes[i].xmax,
                        // msg.bounding_boxes[i].ymax);
                        //printf ("The center pixel of %s is: [row, col] = [%d, %d]\n", object_name.c_str(), object_x, object_y);
                        find_object = true;
                    }
                }
                // 如果遍历了所有的识别框且没有识别出指定的物品
                // 则使用识别出来的第一个物体作为目标
                if (find_object==false && msg.bounding_boxes.size() > 0)
                {
                    object_x = int ((msg.bounding_boxes[0].xmin + msg.bounding_boxes[0].xmax) / 2);
                    object_y = int ((msg.bounding_boxes[0].ymin + msg.bounding_boxes[0].ymax) / 2);
                    find_object = true;
                }
                // 如果并没有识别出来物体
                // 则使用指定位置（321， 381）作为目标点
                else if (msg.bounding_boxes.size() == 0)
                {
                    object_x = int (321);
                    object_y = int (381);
                    find_object = true;
                }
            }
        }
        get_object_position();
    }

    void pclCallback(sensor_msgs::PointCloud2 msg)
    {
        // 机器人到达抓取位置之后开始监听点云变换
        if (in_position == true)
        {
            // 储存原始点云数据
            pcl::fromROSMsg (msg, *cloud_astra);
        }
    }

    void get_object_position()
    {
        if (cloud_astra->empty())
        {
            ROS_INFO ("Waiting for pcl transform");
            sleep (2);
        }
        else
        {
            geometry_msgs::PointStamped cam_pos;
            geometry_msgs::PointStamped base_point;
            cam_pos.header.frame_id = "/astra_depth_optical_frame";
            origin_index = object_y * camera_width + object_x;
            kamerider_image_msgs::ObjectPosition pos;
            tf::TransformListener pListener;

            if (isnan(cloud_astra->points[origin_index].x) || 
                isnan(cloud_astra->points[origin_index].y) ||
                isnan(cloud_astra->points[origin_index].z))
            {  
                int new_index = find_near_valid(origin_index);
                cam_pos.header.stamp = ros::Time(0);
                cam_pos.point.x = cloud_astra->points[new_index].x;
                cam_pos.point.y = cloud_astra->points[new_index].y;
                cam_pos.point.z = cloud_astra->points[new_index].z;
				
                try
                {
                    pListener.waitForTransform("/astra_depth_optical_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
                    pListener.transformPoint("/base_link", cam_pos, base_point);
                    ROS_INFO("cam_point: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                            cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
                            base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("Received an exception trying to transform a point from \"astra_depth_optical_frame\" to \"base_link\": %s", ex.what());
                }
                
            }
            else
            {
                cam_pos.header.stamp = ros::Time(0);
                cam_pos.point.x = cloud_astra->points[origin_index].x;
                cam_pos.point.y = cloud_astra->points[origin_index].y;
                cam_pos.point.z = cloud_astra->points[origin_index].z;
                try
                {
                    pListener.waitForTransform("/astra_depth_optical_frame", "/base_link", ros::Time(0), ros::Duration(3.0));
                    pListener.transformPoint("/base_link", cam_pos, base_point);
                    ROS_INFO("cam_point: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
                            cam_pos.point.x, cam_pos.point.y, cam_pos.point.z,
                            base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
                }
                catch(tf::TransformException& ex)
                {
                    ROS_ERROR("Received an exception trying to transform a point from \"astra_depth_optical_frame\" to \"base_link\": %s", ex.what());
                }
            }
            if (!pub_obj_position)
            {
                // 控制发布器只发布一次消息
                pos.header.frame_id = "/base_link";
                pos.x = base_point.point.x;
                pos.y = base_point.point.y;
                pos.z = base_point.point.z;
                obj_pub.publish (pos);
                pub_obj_position = true;
            }
        }
    }

public:
    int run (int argc, char** argv)
    {
        ROS_INFO("----------INIT----------");
        ros::init (argc, argv, "object_detection");
        ros::NodeHandle nh;
        ROS_INFO("----Waiting for image----");

        nh.param<std::string>("sub_bbox_topic_name",       sub_bbox_topic_name,       "/darknet_ros/bounding_boxes");
        nh.param<std::string>("sub_pcl_topic_name",        sub_pcl_topic_name,        "/astra/depth/points");
        nh.param<std::string>("sub_image_raw_topic_name",  sub_image_raw_topic_name,  "/astra/rgb/image_raw");
        nh.param<std::string>("sub_control_topic_name",    sub_control_topic_name,    "/control_to_image");

        nh.param<std::string>("pub_object_pos_topic_name", pub_object_pos_topic_name, "/kamerider_image/object_position_base");
        nh.param<std::string>("pub_tf_pcl_topic_name",     pub_tf_pcl_topic_name,     "/base_link_pcl");

        pcl_sub = nh.subscribe(sub_pcl_topic_name, 1, &object_detection::pclCallback, this);
        bbox_sub = nh.subscribe(sub_bbox_topic_name, 1, &object_detection::darknetCallback, this);
        img_sub = nh.subscribe(sub_image_raw_topic_name, 1, &object_detection::imageCallback, this);
        control_sub = nh.subscribe(sub_control_topic_name, 1, &object_detection::controlCallback, this);
        
        obj_pub = nh.advertise<kamerider_image_msgs::ObjectPosition>(pub_object_pos_topic_name, 1);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(pub_tf_pcl_topic_name, 1);
        
        std::cout << "Receving message from topics: " << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "\t" << sub_bbox_topic_name << std::endl;
        std::cout << "\t" << sub_pcl_topic_name << std::endl;
        std::cout << "\t" << sub_control_topic_name << std::endl;
        std::cout << "\t" << sub_image_raw_topic_name << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "Publishing message to topics: " << std::endl;
        std::cout << "--------------------------" << std::endl;
        std::cout << "\t" << pub_object_pos_topic_name << std::endl;
        std::cout << "\t" << pub_tf_pcl_topic_name << std::endl;
        std::cout << "--------------------------" << std::endl;

        ros::spin();
    }

};

int main(int argc, char** argv)
{
    object_detection detector;
    return detector.run(argc, argv);
}

