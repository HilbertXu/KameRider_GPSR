#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/03/07
    Author: Xu Yucheng
    Abstract: speech recognition codes with xfei api
"""

import os
import sys
import time
import requests
import json
import hashlib
import base64
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8

class speech_recognition_xfei():
    def __init__(self):
        self.URL = "http://api.xfyun.cn/v1/service/v1/iat"
        self.APPID = "5c8102a7"
        self.API_KEY = "cd41a5bfe6b542e1e4fc8edac15a3c88"
        self.audio_folder = "/home/nvidia/catkin_ws/src/kamerider_speech/sounds/gpsr_record/"
        self.header = None
        self.config = None

        self.sub_audio_topic_name = None
        self.pub_recognition_result_topic_name = None
        self.pub_result = None
        self.is_restart = None
        self.get_params()
        self.get_header()
        # 由参数服务器中的参数判断当前是否是重启节点
        if self.is_restart != False:
            self.restart()
        else:
            os.chdir(self.audio_folder)
            os.system("rm -rf *")
    
    def restart(self):
        # 当讯飞因为使用时间过长而Timeout之后，通过rosnode kill和launch文件中的respawn选项重启这个节点
        # 重启这个节点之后需要读取刚刚没有识别出来的语音文件再次识别
        print ("Node restart DETECTED!")
        file_num = len(os.listdir(self.audio_folder))
        path_to_wav = self.audio_folder + 'gpsr_' + str(file_num-1) + '.wav'
        self.get_wav(path_to_wav)

    
    def get_params(self):
        # ROS params
        self.is_restart = rospy.get_param("/is_restart", "")
        self.sub_audio_topic_name = rospy.get_param("sub_audio_topic_name", "/audio_index")
        self.pub_recognition_result_topic_name = rospy.get_param("pub_recognition_result_topic_name", "/xfei_output")
        # ROS subscriber & publisher
        rospy.Subscriber(self.sub_audio_topic_name, Int8, self.audioCallback)
        self.pub_result = rospy.Publisher(self.pub_recognition_result_topic_name, String, queue_size=1)
    
    def get_header(self, aue='raw', engineType='sms-en16k'):
        cur_time = str(int(time.time()))
        param = "{\"aue\":\"" + aue + "\"" + ",\"engine_type\":\"" + engineType + "\"}"
        print ("param: {}".format(param))
        try:
            param_base64 = str(base64.b64encode(param.encode('utf-8')), 'utf-8')
        except:
            param_base64 = str(base64.b64encode(param.encode('utf-8')))

        print ("x_param: {}".format(param_base64))

        m2 = hashlib.md5()
        self.config = self.API_KEY + cur_time + param_base64
        m2.update(self.config.encode('utf-8'))
        check_sum = m2.hexdigest()
        print ("check_sum: {}".format(check_sum))
        self.header = {
            'X-CurTime': cur_time,
            'X-Param': param_base64,
            'X-Appid': self.APPID,
            'X-CheckSum': check_sum,
            'Content-Type': 'application/x-www-form-urlencoded; charset=utf-8',
        }
        print (self.header)
        
    def audioCallback(self, msg):
        audio_index = str(msg.data)
        path_to_wav = self.audio_folder + "gpsr_" + audio_index + ".wav"
        self.get_wav(path_to_wav)

    def get_wav(self, path):
        bin_file = open(path, 'rb')
        data = {'audio': base64.b64encode(bin_file.read())}
        
        result = requests.post(self.URL, headers=self.header, data=data)
        print (result.content.decode('utf-8'))
        result_dict = json.loads(result.content.decode('utf-8'))
        if str(result_dict["code"]) == str(10114):
            rospy.set_param("/is_restart", "true")
            os.system("rosnode kill /speech_recognition")
        else:
            string = result_dict["data"]
            self.pub_result.publish(string)

if __name__ == '__main__':
    rospy.init_node('speech_recognition')
    xfei = speech_recognition_xfei()
    rospy.spin()

