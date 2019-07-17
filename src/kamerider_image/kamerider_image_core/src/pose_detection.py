#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2019/02/03
Author: Xu Yucheng
Abstract: pose detect code with openpose
'''
import os
import sys
import math
import cv2
import rospy
import roslib
import base64
import numpy as np
from std_msgs.msg import Int8
from std_msgs.msg import String 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from kamerider_control_msgs.msg import Mission, Result
# openpose module
from openpose import pyopenpose as op

IMAGE_DIR = "../result/"

class pose_detection:
    def __init__(self):
        self.params = dict()
        self.target_pose = None
        self.start_pose_detect = False
        self.take_photo = False
        self.pose_detect_finish = False
        self.sub_image_raw_topic_name = None
        self.sub_control_back_topic_name = None
        self.pub_to_control_topic_name=None
        self.pub_pose_detect_result_topic_name = None
        self.pub_to_speech = None
        self.set_param()

        # start openpose
        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure(self.params)
        self.opWrapper.start()
    
    def set_param(self):
        # ROS param参数服务器中取得参数值
        self.sub_image_raw_topic_name             = rospy.get_param('sub_image_raw_topic_name',          '/astra/rgb/image_raw')
        self.sub_control_back_topic_name          = rospy.get_param('sub_control_back_topic_name',       '/control_to_image')

        self.pub_to_speech_topic_name             = rospy.get_param("pub_to_speech",                     '/input_to_control')
        self.pub_to_control_topic_name            = rospy.get_param("pub_to_control_topic_name",         '/image_to_control')

        # 发布器和订阅器
        self.sub_image = rospy.Subscriber(self.sub_image_raw_topic_name, Image, self.image_callback)
        self.sub_control = rospy.Subscriber(self.sub_control_back_topic_name, Mission, self.control_callback)

        self.pub_speech = rospy.Publisher(self.pub_to_speech_topic_name, String, queue_size=1)
        self.pub_control = rospy.Publisher(self.pub_to_control_topic_name, Result, queue_size=1)
        
        # 设置openpose手部特征点检测时的参数
        self.params["model_folder"] = "/home/kamerider/openpose/models"

    def control_callback(self, msg):
        if msg.mission_type == "gesture":
            self.target_pose = msg.mission_name
            rospy.loginfo("start to take a photo")
            self.take_photo = True
            rospy.loginfo("received target pose "+self.target_pose)
    
    def image_callback(self, msg):
        if self.take_photo:
            bridge = CvBridge()
            try:
                cv_image = bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.pose_detect(cv_image)
                self.take_photo = False
            except CvBridgeError as e:
                print (e)
    
    def pose_detect(self, cv_image):
        # Create new datum
        datum = op.Datum()
        datum.cvInputData = cv_image
        # Process and display image
        self.opWrapper.emplaceAndPop([datum])
        # Display Image
        print("Body keypoints: \n" + str(datum.poseKeypoints))
        print("Face keypoints: \n" + str(datum.faceKeypoints))
        cv2.imwrite(IMAGE_DIR + "pose_detect_result.png", datum.cvOutputData)
        self.parse_pose(datum.poseKeypoints)
    
    def parse_pose(self, pose_points):
        #     {0,  "Nose"},
        #     {1,  "Neck"},
        #     {2,  "RShoulder"},
        #     {3,  "RElbow"},
        #     {4,  "RWrist"},
        #     {5,  "LShoulder"},
        #     {6,  "LElbow"},
        #     {7,  "LWrist"},
        #     {8,  "MidHip"},
        #     {9,  "RHip"},
        #     {10, "RKnee"},
        #     {11, "RAnkle"},
        #     {12, "LHip"},
        #     {13, "LKnee"},
        #     {14, "LAnkle"},
        #     {15, "REye"},
        #     {16, "LEye"},
        #     {17, "REar"},
        #     {18, "LEar"},
        #     {19, "LBigToe"},
        #     {20, "LSmallToe"},
        #     {21, "LHeel"},
        #     {22, "RBigToe"},
        #     {23, "RSmallToe"},
        #     {24, "RHeel"},
        #     {25, "Background"}
        count = 1
        for pose_point in pose_points:
            result_pose=[]
            self.pub_speech.publish("now i start detecting the gesture of person {}".format(count))
            # 先判断每一个识别出来的人的gesture
            # 首先判断人是否躺在地上（不脏么。。。）
            # 判断Neck和MidHip 之间y轴方向上的距离和x轴方向上的距离大小
            if (abs(pose_point[1][0] - pose_point[8][0]) > abs(pose_point[1][1] - pose_point[8][1])):
                rospy.loginfo("The person in front of me is lying on the ground")
                result_pose.append("lying")

            # 然后判断是站着还是坐着
            # 由头部在画面中的位置判断
            elif pose_point[0][1] > 120:
                rospy.loginfo("The person in front of me is sitting")
                result_pose.append("sitting")
                if (pose_point[6][1] < pose_point[5][1]) or (pose_point[7][1] < pose_point[6][1]) or (pose_point[3][1] < pose_point[2][1]) or (pose_point[4][1] < pose_point[3][1]):
                    if (pose_point[6][1] < pose_point[5][1]) or (pose_point[7][1] < pose_point[6][1]):
                        rospy.loginfo("The person in front of me is raising his left hand")
                        result_pose.append("raising left")

                    if (pose_point[3][1] < pose_point[2][1]) or (pose_point[4][1] < pose_point[3][1]):
                        rospy.loginfo("The person in front of me is raising his right hand")
                        result_pose.append("raising right")
                # 然后判断是不是指向左边或者右边
                # 判断这个人是指向左边还是右边
                else:
                    if (abs(pose_point[6][0] - pose_point[5][0]) > abs(pose_point[3][0] - pose_point[2][0])):
                        rospy.loginfo("The person in front of me is pointing to the left")
                        result_pose.append("pointing left") 

                    if (abs(pose_point[6][0] - pose_point[5][0]) < abs(pose_point[3][0] - pose_point[2][0])):
                        rospy.loginfo("The person in front of me is pointing to the right")
                        result_pose.append("pointing right")
                print ("result pose: {}".format(result_pose))
                print ("target pose: {}".format(self.target_pose))

            # 判断出来是站着的人
            elif pose_point[0][1] < 120:
                # 首先判断是不是举手
                # 然后判断这个人是举左手还是举右手
                result_pose.append("standing")
                if (pose_point[6][1] < pose_point[5][1]) or (pose_point[7][1] < pose_point[6][1]) or (pose_point[3][1] < pose_point[2][1]) or (pose_point[4][1] < pose_point[3][1]):
                    if (pose_point[6][1] < pose_point[5][1]) or (pose_point[7][1] < pose_point[6][1]):
                        rospy.loginfo("The person in front of me is raising his left hand")
                        result_pose.append("raising left")

                    if (pose_point[3][1] < pose_point[2][1]) or (pose_point[4][1] < pose_point[3][1]):
                        rospy.loginfo("The person in front of me is raising his right hand")
                        result_pose.append("raising right")
                # 然后判断是不是指向左边或者右边
                # 判断这个人是指向左边还是右边
                else:
                    if (abs(pose_point[6][0] - pose_point[5][0]) > abs(pose_point[3][0] - pose_point[2][0])):
                        rospy.loginfo("The person in front of me is pointing to the left")
                        result_pose.append("pointing left") 

                    if (abs(pose_point[6][0] - pose_point[5][0]) < abs(pose_point[3][0] - pose_point[2][0])):
                        rospy.loginfo("The person in front of me is pointing to the right")
                        result_pose.append("pointing right")
                print ("result pose: {}".format(result_pose))
                print ("target pose: {}".format(self.target_pose))
            count += 1
            

        if self.target_pose != "none":
            if self.target_pose in result_pose:
                msg = String()
                msg.data = "I have found the person {}".format(self.target_pose)
                control_msg = Result()
                control_msg.mission_type = "gesture"
                control_msg.result = "success"
                self.pub_speech.publish(msg)
                self.pub_control.publish(control_msg)
            else:
                msg = String()
                msg.data = "sorry i cannot found the person {}".format(self.target_pose)
                control_msg = Result()
                control_msg.mission_type = "gesture"
                control_msg.result = "fail"
                self.pub_speech.publish(msg)
                self.pub_control.publish(control_msg)

        if self.target_pose == "none":
            string= String()
            control_msg = Result()
            control_msg.mission_type = "gesture"
            if len(result_pose) > 1:
                string.data = "i have found a {} person {}".format(result_pose[0], result_pose[1])                 
                if 'raising left' in result_pose or 'raising right' in result_pose:                    
                    control_msg.result = result_pose[0] + ' and ' + result_pose[1] + ' hand'
                else:
                    control_msg.result = result_pose[0] + ' and ' + result_pose[1]
            elif len(result_pose) == 1:
                string.data = "i have found a {} person".format(result_pose[0])
                if 'raising left' in result_pose or 'raising right' in result_pose:                    
                    control_msg.result = result_pose[0] + ' hand'
                else:
                    control_msg.result = result_pose[0]
            self.pub_speech.publish(string)
            self.pub_control.publish(control_msg)
            rospy.sleep(2)               
    
if __name__ == '__main__':
    rospy.init_node("pose_detection")
    detector = pose_detection()
    rospy.spin()


	
