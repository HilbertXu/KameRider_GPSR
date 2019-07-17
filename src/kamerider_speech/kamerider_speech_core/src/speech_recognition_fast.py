#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
    Date: 2019/03/07
    Author: Xu Yucheng
    Abstract: speech recognition codes with baidu api
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

IS_PY3 = sys.version_info.major == 3

if IS_PY3:
    from urllib.request import urlopen
    from urllib.request import Request
    from urllib.error import URLError
    from urllib.parse import urlencode
    timer = time.perf_counter
else:
    from urllib2 import urlopen
    from urllib2 import Request
    from urllib2 import URLError
    from urllib import urlencode
    if sys.platform == "win32":
        timer = time.clock
    else:
        # On most other platforms the best timer is time.time()
        timer = time.time
class DemoError(Exception):
    pass

TOKEN_URL = 'http://openapi.baidu.com/oauth/2.0/token'
SCOPE = 'audio_voice_assistant_get'  # 有此scope表示有asr能力，没有请在网页里勾选
class speech_recognition_baidu():
    def __init__(self):
        self.URL = "http://api.xfyun.cn/v1/service/v1/iat"
        self.APPID = "15944331"
        self.API_KEY = "bpNdxEhagyalZVtC6fddFeGZ"
        self.SECRET_KEY = '1QRtmDnXAA9TUQuGH8zHL5K1OW2GnpDu'
        self.audio_folder = "/home/kamerider/kamerider_GPSR/src/kamerider_speech/kamerider_speech_core/sounds/gpsr_record/"
        self.ASR_URL = 'http://vop.baidu.com/server_api'
        self.DEV_PID = 1737
        self.CUID = '123456PYTHON'
        self.FORMAT = 'wav'
        self.RATE = 16000

        self.sub_audio_topic_name = None
        self.pub_recognition_result_topic_name = None
        self.pub_result = None
        self.is_restart = None
        self.get_params()
        """
        # 由参数服务器中的参数判断当前是否是重启节点
        if self.is_restart != False:
            self.restart()
        else:
            if os.path.isdir(self.audio_folder) == False:
                os.makedirs(self.audio_folder)
            os.chdir(self.audio_folder)
            os.system("rm -rf *")
        """

    def fetch_token(self):
        params = {'grant_type': 'client_credentials',
                'client_id': self.API_KEY,
                'client_secret': self.SECRET_KEY}
        post_data = urlencode(params)
        if (IS_PY3):
            post_data = post_data.encode( 'utf-8')
        req = Request(TOKEN_URL, post_data)
        try:
            f = urlopen(req)
            result_str = f.read()
        except URLError as err:
            result_str = err.read()
        if (IS_PY3):
            result_str =  result_str.decode()

        result = json.loads(result_str)
        if ('access_token' in result.keys() and 'scope' in result.keys()):
            if not SCOPE in result['scope'].split(' '):
                raise DemoError('scope is not correct')
            print('SUCCESS WITH TOKEN: %s ; EXPIRES IN SECONDS: %s' % (result['access_token'], result['expires_in']))
            return result['access_token']
        else:
            raise DemoError('MAYBE API_KEY or SECRET_KEY not correct: access_token or scope not found in token response')

    
    def get_params(self):
        # ROS params
        self.is_restart = rospy.get_param("/is_restart", "")
        self.sub_audio_topic_name = rospy.get_param("sub_audio_topic_name", "/audio_index")
        self.pub_recognition_result_topic_name = rospy.get_param("pub_recognition_result_topic_name", "/baidu_to_control")
        # ROS subscriber & publisher
        rospy.Subscriber(self.sub_audio_topic_name, Int8, self.audioCallback)
        self.pub_result = rospy.Publisher(self.pub_recognition_result_topic_name, String, queue_size=1)
    
    def audioCallback(self, msg):
        rospy.set_param("wait_for_command", False)
        token = self.fetch_token()
        audio_index = str(msg.data)
        path_to_wav = self.audio_folder + "gpsr_" + audio_index + ".wav"

        speech_data = []
        with open(path_to_wav, 'rb') as speech_file:
            speech_data = speech_file.read()
        length = len(speech_data)
        speech = base64.b64encode(speech_data)
        if (IS_PY3):
            speech = str(speech, 'utf-8')
        params = {'dev_pid': self.DEV_PID,
                'format': self.FORMAT,
                'rate': self.RATE,
                'token': token,
                'cuid': self.CUID,
                'channel': 1,
                'speech': speech,
                'len': length
                }  
        post_data = json.dumps(params, sort_keys=False)
        # print post_data
        req = Request(self.ASR_URL, post_data.encode('utf-8'))
        req.add_header('Content-Type', 'application/json')
        try:
            begin = timer()
            f = urlopen(req)
            result_str = f.read()
            print (result_str)
        except  URLError as err:
            print ("Owwwwww something bad happened")
        
        result_dict = json.loads(result_str)
        if "result" not in result_dict.keys():
            self.pub_result.publish("failed")
        else:
            msg = String()
            msg.data = str(result_dict["result"][0])
            self.pub_result.publish(msg)
        rospy.set_param("wait_for_command", True)

if __name__ == '__main__':
    rospy.init_node('speech_recognition_baidu')
    baidu = speech_recognition_baidu()
    rospy.spin()

