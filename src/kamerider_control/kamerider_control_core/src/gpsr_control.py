#! /usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
'''
roslib.load_manifest ('speech')
roslib.load_manifest () reads the package manifest and sets up the python library path based on the package dependencies. 
It's required for older rosbuild-based packages, but is no longer needed on catki
'''
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
import os
import sys
import time
import wave
import datetime
import pyaudio
from kamerider_control_msgs.msg import Mission
from kamerider_control_msgs.msg import Result
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main
from play_signal_sound import play_signal_sound
from answer_question import answer_question
from speech_rec_correction import speech_rec_correction
from kamerider_image_msgs.msg import GenderDetection
# 定义表示任务状态的变量
UNSTART = 0
PROCESS = 1
FAILED  = 2
FINISH  = 3

class gpsr_speech_control(object):
    """Class to read the recognition output of pocketsphinx"""
    #一些参数的初始化
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        # Default infomations
        _, self.locations, self.objects = read_main()
        rospy.loginfo("GPSR control online~")
        # Type of task
        self.task_type = None

        # Predefined missions
        self.location = []
        for key in self.locations.keys():
            for loc in self.locations[key]:
                self.location.append(loc)

        self.object = self.dict_to_list(self.objects)
        self.action=['bring', 'take','took','put','give','gave','deliver','delivered','place','pick']
        self.people=['person','people','girl','boy','male','female']
        self.name=['Alex','Charlie','Elizabeth','Francis','Jennifer','Linda','Mary','Patricia','Robin','Skyler','Alex','Charlie','Francis','James','John','Michael','Robert', 'Robin','Skyler','William']
        self.gender=['boy','girl','man','woman','male','famale']
        #self.gesture=['waving','left arm','right arm','pointing to the left','pointing to the right']
        self.pose=['sitting','seeking','standing','lying']
        self.navi=['follow','followed','guide','lead','need','lady','escort','accompany','accompanied','company','meet','meeting']
        self.room=['corridor','bedroom','dining','livingroom','kitchen','bathroom','Exit','operator']
        #self.location=[]
        self.gesture = ['waving', 'raising left', 'raising right', 'pointing left', 'pointing right']
        self.talk = ["affiliation", "relation", "joke", "yourself","day","today"]
        
        # Publisher topics
        self.pub_arm = None
        self.pub_image = None
        self.pub_nav = None
        self.pub_to_nav_topic_name = None
        self.pub_to_image_topic_name = None
        self.pub_to_arm_topic_name = None
        # Subscriber topics
        self.sub_image = None
        self.sub_baidu = None
        self.sub_nav   = None
        self.sub_input = None
        self.sub_arm   = None
        self.sub_baidu_back_topic_name = None
        self.sub_input_back_topic_name = None
        self.sub_nav_back_topic_name = None
        self.sub_image_back_topic_name = None
        self.sub_arm_back_topic_name = None
        # Mission keywords
        self.target_action = []
        self.target_people = []
        self.target_room = []
        self.target_location = []
        self.target_object = []
        self.target_navi = []
        self.target_name = []
        self.target_gender = []
        self.target_gesture = []
        self.target_pose = []
        # Mission keywords##############################
        self._room = UNSTART
        self._navigate = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART
        self._arm = UNSTART
        
        #self.init_params()
        self.get_params()
    
    def dict_to_list(self,Dict):
        List = Dict.keys()
        return List
    
    def cleanup(self):
        rospy.loginfo("See You Next Time")
    
    #表征目前机器人状态的变量（主要是目前在哪里,在执行任务的哪一步）
    def init_params(self):
        # Mission keywords
        self.target_detail = []
        self.target_action = []
        self.target_people = []
        self.target_room = []
        self.target_location = []
        self.target_object = []
        self.target_navi = []
        self.target_name = []
        self.target_gender = []
        self.target_gesture = []
        self.target_pose = []

        # Mission state
        self._room = UNSTART
        self._navigate = UNSTART
        self._person = UNSTART
        self._object = UNSTART
        self._mission = UNSTART
        self._arm = UNSTART
	
        # Person status
        self._person_pose = ''
        self._person_gender = ''

        self.record_switch("on")

    #从launch文件中获取topic_name,声明Publisher和Subscriber,播放初始音频
    def get_params(self):
        # Initialize sound client
        self.sh = SoundClient(blocking=True)
        rospy.sleep(1)
        self.sh.stopAll()
        rospy.sleep(1)

        # Get parameters
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
        self.cmd_files = rospy.get_param("~cmd_file", "/home/nvidia/catkin_ws/src/kamerider_speech/CommonFiles")

        self.sub_image_back_topic_name = rospy.get_param("sub_image_back_topic_name", "/image_to_control")
        self.sub_arm_back_topic_name   = rospy.get_param("sub_arm_back_topic_name", "/arm_to_control")
        self.sub_nav_back_topic_name   = rospy.get_param("sub_nav_back_topic_name", "/nav_to_control")
        self.sub_baidu_back_topic_name = rospy.get_param("sub_baidu_back_topic_name", "/baidu_to_control")
        self.sub_input_back_topic_name = rospy.get_param("sub_input_back_topic_name", "/input_to_control")

        self.pub_to_arm_topic_name   = rospy.get_param("pub_to_arm_topic_name", "/control_to_arm")
        self.pub_to_image_topic_name = rospy.get_param("pub_to_image_topic_name", "/control_to_image")
        self.pub_to_nav_topic_name   = rospy.get_param("pub_to_nav_topic_name", "/control_to_nav")

        self.sub_arm = rospy.Subscriber(self.sub_arm_back_topic_name, Result, self.arm_callback)
        self.sub_image = rospy.Subscriber(self.sub_image_back_topic_name, Result, self.image_callback)
        self.sub_nav   = rospy.Subscriber(self.sub_nav_back_topic_name, Result, self.nav_callback)
        self.sub_baidu = rospy.Subscriber(self.sub_baidu_back_topic_name, String, self.baidu_callback)
        # 由外部节点输入到control节点的语音说话的内容
        self.sub_input = rospy.Subscriber(self.sub_input_back_topic_name, String, self.input_callback)

        self.pub_arm   = rospy.Publisher(self.pub_to_arm_topic_name, Mission, queue_size=1)
        self.pub_image = rospy.Publisher(self.pub_to_image_topic_name, Mission, queue_size=1)
        self.pub_nav   = rospy.Publisher(self.pub_to_nav_topic_name, Mission, queue_size=1)

        # Start gpsr task
        self.start_gpsr()

    def publish_message(self, pub, msg):
        pub.publish(msg)

    def record_switch(self, target):
        # 通过控制参数服务器（ros param）中的wait_for_command来控制录音节点的开关
        if target == "on":
            rospy.set_param("/wait_for_command", True)
        elif target == "off":
            rospy.set_param("/wait_for_command", False)

    def image_process(self, Type, name="none"):
        msg = Mission()
        msg.mission_type = Type
        msg.mission_name = name
        if Type == "person":
            if self._person != PROCESS:
                self.publish_message(self.pub_image, msg)
                self._person = PROCESS

        if Type == "object":
            if self._object != PROCESS:
                self.publish_message(self.pub_image, msg)
                self._object = PROCESS

        if Type == "gesture":
            if self._mission != PROCESS:
                self.publish_message(self.pub_image, msg)
                self._mission = PROCESS

        if Type == "gender":
            if self._mission != PROCESS:
                self.publish_message(self.pub_image, msg)
                self._mission = PROCESS

    def navigate_to(self, location):
        msg = Mission()
        msg.mission_type = "navigate"
        msg.mission_name = location
        if self._navigate != PROCESS:
            self.publish_message(self.pub_nav, msg)
    
    def arm_manipulate(self, target):
        msg = Mission()
        msg.mission_type = "arm"
        msg.mission_name = target
        if self._arm != PROCESS:
            self.publish_message(self.pub_arm, msg)
            self._arm = PROCESS

    def input_callback(self, msg):
        Str = msg.data
        self.sh.say(Str, self.voice)

    def image_callback(self, msg):
        if msg.mission_type == "person":
            if msg.result == "success":
                self._person = FINISH

        if msg.mission_type == "object":
            if msg.result == "success":
                self._object = FINISH
        
        if msg.mission_type == "gesture":
            # 此处是对寻找指定姿态的回调进行判断的部分
            # 只存在找到指定姿态(success) 和 没有找到指定姿态(fail)两种情况
            if msg.result == "success":
                self._mission = FINISH
            if msg.result == "fail":
                # 如果没有找到指定的姿态，则重启找人这一步骤，对找到的下一个人判断姿态
                rospy.loginfo("Target Gesture Not Found, Restart Mission")
                self._person = UNSTART
                self._mission = FAILED
            # 此处是对指定房间内操作者姿态判断的回调部分
            # msg.result 中直接返回了操作者的姿态
            elif msg.result != "none":
                self._person_pose = msg.result
                self._mission = FINISH
    
        if msg.mission_type == "gender":
            # 此处是对寻找指定性别的回调进行判断的部分
            # 只存在找到指定性别(success) 和 没有找到指定性别(fail)两种情况
            if msg.result == "success":
                self._mission = FINISH
            if msg.result == "fail":
                # 如果没有找到指定的性别，则重启找人这一步骤，对找到的下一个人判断性别
                rospy.loginfo("Target Gender Not Found, Restart Mission")
                self._person = UNSTART
                self._mission = FAILED
            # 此处是对指定房间内操作者性别判断的回调部分
            # msg.result 中直接返回了操作者的性别
            elif msg.result != "none":
                self._person_gender = msg.result
                self._mission = FINISH
    
    def nav_callback(self, msg):
        if msg.mission_type == "navigate":
            if msg.result == "success":
                self._navigate = FINISH
    
    # @TODO
    # 修改机械臂抓取步骤为 伸出， 抓取， 释放
    # 此处用于需要将物体带回给操作者的一类任务
    # 待补充这一类任务的循环部分
    def arm_callback(self, msg):
        if msg.mission_type == "arm":
            if msg.result == "success":
                self._arm = FINISH

    def baidu_callback(self, msg):
        if msg.data == "failed":
            self.sh.say("Sorry i cannot understand your question please speak again", self.voice)
        string = msg.data
        symbols = ["!", "?", ".", ",", ";", ":"]
        output = []
        for part in string.lstrip().split(","):
            for word in part.split():
                for symbol in symbols:
                    if symbol in word:
                        word = word[:-1] #去除了可能出现的句子中间的标点符号
                output.append(word)
        output = [item.lower() for item in output]
        print (output)
        speech_rec_correction(output)
        print (output)
        self.init_params()
        self.parse_output(output)
    
    
    # 播放初始化的音频,并提醒操作者如何使用语音操作Jack
    def start_gpsr(self):
        rospy.sleep(20)
        self.sh.say("the door is open, i will enter the room now", self.voice)
        rospy.sleep(2)
        self.navigate_to("operator")
        while(True):
            if self._navigate == FINISH:
                self.sh.say("Hello my name is Jack", self.voice)
                self.sh.say("Please say Jack to wake me up before each question", self.voice)
                self.sh.say("I am ready for your command if you hear", self.voice)
                play_signal_sound()
                self.init_params()
                break

    def read_default_location(self, target_object):
        default_loc = None
        # 由物体名称从xml文件中读取文件的默认位置
        for key in self.objects.keys():
            if key == target_object:
                default_loc = self.objects[key]["location"]
        return default_loc
        
    # 首先收集输出中的各类关键词
    def parse_output(self, output):
        for room in self.room:
            if room in output:
                self.target_room.append(room)
        for location in self.location:
            if location in output or location+'s' in output:
                self.target_location.append(location)
        for person in self.people:
            if person in output:
                self.target_people.append(person)
        for name in self.name:
            if name in output:
                self.target_name.append(name)
        for Object in self.object:
            if Object in output or Object+'s' in output:
                self.target_object.append(Object)
        for action in self.action:
            if action in output:
                self.target_action.append(action)
        for navi in self.navi:
            if navi in output:
                self.target_navi.append(navi)
        for gender in self.gender:
            if gender in output:
                self.target_gender.append(gender)
        for gesture in self.gesture:
            # Gesture 匹配存在问题
            if gesture in output:
                self.target_gesture.append(gesture)
        for pose in self.pose:
            if pose in output:
                self.target_pose.append(pose)
        for word in self.talk:
            if word in output:
                self.target_detail.append(word)
        
        # 根据上面分类的情况,将问题分成5类
        # ["manipulation", "navigation", "people", "object", "Q&A"]
        # 一旦能够成功地将问题分类，则关闭录音节点, 防止在执行任务的过程中误识别引起错误
        self.target_location = self.target_location + self.target_room
        object_location = self.read_default_location(self.target_location)
        if self.target_action  and self.target_object:
            #manipulation
            self.task_type = "manipulation"
            print("task type is {}".format(self.task_type))
            if self.target_people:
                if 'me' in output:
                    #find obj
                    #grasp
                    #go back
                    # 关闭录音节点
                    self.record_switch("off")
                    print("go to find {} and bring it to me".format(self.target_object))
                    self.navigate_to(object_location)
                    while (True):
                        if self._person == FINISH:
                            # 到达指定地点之后
                            rospy.loginfo("Start finding object")
                            self.sh.say("Now start findfing object, this may take a long moment, please wait", self.voice)
                            os.system("sh /home/nvidia/catkin_ws/start_manipulation.sh")
                            rospy.sleep(20)
                            self.image_process("object" ,self.target_object[0])
                            break
                    while (True):
                        if self._object == FINISH:
                            self.sh.say("I have grasped the target object, now i will return to the start point", self.voice)
                            # @TODO 更换为start point
                            self.navigate_to("operator")
                            break
                    while (True):
                        if self._navigate == FINISH:
                            self.sh.say("I have back to the operator, here is the target object", self.voice)
                            self.arm_manipulate("release")
                            rospy.sleep(8)
                            os.system("sh /home/nvidia/catkin_ws/kill_manipulation.sh")
                            rospy.sleep(3)
                            break
                    self.init_params()
                else:
                    #find obj
                    #grasp
                    #find person
                    #give it to the person
                    # 关闭录音节点
                    self.record_switch("off")
                    print("go to find {} and give it to the person who's feature is {}".format(self.target_object,self.target_gender+self.target_gesture+self.target_pose))
                    gesture = ''
                    if self.target_gesture:
                        gesture = self.target_gesture[0]
                    if self.target_pose:
                        gesture = self.target_pose[0]
                        
                    self.navigate_to(object_location)
                    while (True):
                        if self._navigate == FINISH:
                            rospy.loginfo("target location arrived")
                            rospy.loginfo("Start find object")
                            self.sh.say("Now start finding object, this may take a moment, please wait", self.voice)
                            os.system("sh /home/nvidia/catkin_ws/start_manipulation.sh")
                            rospy.sleep(20)
                            self.image_process("object", self.target_object[0])
                            break
                    while (True):
                        if self._object == FINISH:
                            rospy.loginfo("start find person")
                            self.sh.say("Now start find person {}".format(gesture), self.voice)
                            os.system("rosrun kamerider_image_core person_detection.py")
                            rospy.sleep(5)
                            self.image_process("person")
                            break
                    while (True):
                        # 如果没有找到指定特征的人，则将_person置为UNSTART，然后重新开始找人
                        if self._person == UNSTART:
                            self.image_process("person")
                        # 目前只考虑pose和gender， 姓名暂时无法完成
                        # 如果当前任务状态(_mission)为UNSTART, 表明还没有开启gesture识别的节点
                        # 开启节点，并且发布消息进行识别
                        if self._person == FINISH and (self.target_gesture or self.target_pose) and self._mission == UNSTART:
                            rospy.loginfo("Start gender recognition")
                            rospy.sleep(3)
                            os.system("rosnode kill /person_detection")
                            os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core pose_detection.py'")
                            rospy.sleep(5)
                            self.image_process("gesture", gesture)

                        if self._person == FINISH and self.target_gender and self._mission == UNSTART:
                            rospy.loginfo("Start gender recognition")
                            rospy.sleep(3)
                            os.system("rosnode kill /person_detection")
                            os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core gender_recognition.py'")
                            rospy.sleep(3)
                            self.image_process("gender", self.target_gender)

                        # 如果任务状态(_mission)为PROCESS， 表明节点已经开启，但是没有找到指定的gesture
                        # 此时如果_person为FINISH， 表明已经找到了下一个人，
                        if self._person == FINISH and self.target_gesture and self._mission == PROCESS:
                            rospy.loginfo("Start pose recognition")
                            self.image_process("pose", gesture)

                        if self._person == FINISH and self.target_gender and self._mission == PROCESS:
                            rospy.loginfo("Start gender recognition")
                            self.image_process("gender", gesture)

                    self.navigate_to("operator")
                    while (True):
                        if self._navigate == FINISH:
                            self.sh.say("I have found the target person, here is the target object", self.voice)
                            self.arm_manipulate("release")
                            rospy.sleep(8)
                            os.system("sh /home/nvidia/catkin_ws/kill_manipulation.sh")
                            break
                    self.init_params() 
            else:
                #find obj
                #grasp
                #go to location[0] if not none
                #place obj
                if self.target_location:
                    # 关闭录音节点
                    self.record_switch("off")
                    print("go to find {} and put it at {}".format(self.target_object,self.target_location[-1]))
                    self.navigate_to(object_location)
                    while (True):
                        if self._navigate == FINISH:
                            rospy.loginfo("Target object location reached")
                            self.sh.say("Start finding object, this may take a moment, please wait", self.voice)
                            os.system("sh /home/nvidia/catkin_ws/start_manipulation.sh")
                            rospy.sleep(20)
                            self.image_process("object")
                            break
                    while (True):
                        if self._object == FINISH:
                            rospy.loginfo("I have get the target object, now i will go to".format(self.target_location[0]))
                            self.sh.say("I have get the target object, now i will go to".format(self.target_location[0], self.voice))
                            os.system("sh /home/nvidia/catkin_ws/kill_manipulation.sh")
                            rospy.sleep(4)
                            # @TODO
                            self.navigate_to(self.target_location[0])
                            break
                    while (True):
                        if self._navigate == FINISH:
                            rospy.loginfo("{} reached".format(self.target_location[0]))
                            self.arm_manipulate("release")
                            rospy.sleep(4)
                            break
                    self.navigate_to("operator")
                    while (True):
                        if self._navigate == FINISH:
                            self.init_params()
                            self.sh.say("okay i am ready for your next command", self.voice)
                            break
        else:
            if self.target_navi and self.target_location:
                #follow&guige
                self.task_type = "navigation"
                print("task type is {}".format(self.task_type))
                #把room和location当做同样的性质处理
                if len(self.target_location) ==1:
                    if 'follow' in output:
                        print("go to {}".format(self.target_location[0]))
                        print("find person start follow until \'stop\'")
                        # 关闭录音节点
                        self.record_switch("off")
                        #go to lacation[0]
                        self.sh.say("Now I start navigation", self.voice)
                        self.navigate_to(self.target_location[0])
                        #find person
                        while(True):
                            if self._navigate == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                                rospy.sleep(5)
                                self.image_process("person")
                                break
                        #start follow until hear stop
                        while (True):
                            if self._person == FINISH:
                                rospy.loginfo("Find person finished start to follow")
                                rospy.sleep(3)
                                os.system("rosnode kill /person_detection")
                                os.system("gnome-terminal -x bash -c 'roslaunch turtlebot_follower follower.launch'")
                                self.sh.say("Please stand in front of me and lead me", self.voice)
                                self.sh.say("I will stop after hearing jack stop")
                                rospy.sleep(2)
                                # @TODO
                                # 写一个新的follower control
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_control_core follower_control.py'")
                                break
                        self.navigate_to("operator")
                        while (True):
                            if self._navigate == FINISH:
                                self.init_params()
                                self.sh.say("okay i am ready for your next command", self.voice)
                                break
                        self.init_params()
                    else:
                        print("find person and start navi to {}".format(self.target_location[0]))
                        # 关闭录音节点
                        self.record_switch("off")
                        #find person and start navi to location[0]
                        rospy.loginfo("Start finding person")
                        self.sh.say("Now start finding person, please wait", self.voice)
                        os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                        rospy.sleep(5)
                        self.image_process("person")
                        #go to lacation[0]
                        while (True):
                            if self._person ==FINISH:
                                self.sh.say("Now I start navigation,please follow me",self.voice)
                                self.navigate_to(self.target_location[0])
                                break
                        while (True):
                            if self._navigate == FINISH:
                                self.sh.say("I have arrived at {}".format(self.target_location[0]), self.voice)
                                self.sh.say("now i will go back to the operator", self.voice)
                                rospy.sleep(2)
                                break
                        self.navigate_to("operator")
                        while (True):
                            if self._navigate == FINISH:
                                self.init_params()
                                self.sh.say("okay i am ready for your next command", self.voice)
                                break
                        self.init_params()

                if len(self.target_location) ==2:
                    if output.index(self.target_location[0]) > output.index(self.target_location[1]):
                        self.target_location.reverse()
                    if 'find' in output or 'found' in output:
                        self.target_location.reverse()                        
                    if 'follow' in output:
                        # go to loc[0]
                        print("go to {} and start follow until \'stop\'".format(self.target_location[0]))
                        # 关闭录音节点
                        self.record_switch("off")
                        self.sh.say("Now I start navigation", self.voice)
                        self.navigate_to(self.target_location[0])
                        while (True):
                            if self._navigate == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                                rospy.sleep(5)
                                self.image_process("person")
                                break
                        while (True):
                            if self._person == FINISH:
                                rospy.loginfo("Find person finished start to follow")
                                rospy.sleep(3)
                                os.system("rosnode kill /person_detection")
                                os.system("gnome-terminal -x bash -c 'roslaunch turtlebot_follower follower.launch'")
                                self.sh.say("Please stand in front of me and lead me", self.voice)
                                self.sh.say("I will stop after hearing jack stop")
                                rospy.sleep(2)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_control_core follower_control.py'")
                                break
                        self.navigate_to("operator")
                        while (True):
                            if self._navigate == FINISH:
                                self.init_params()
                                self.sh.say("okay i am ready for your next command", self.voice)
                                break
                        self.init_params()

                    else:
                        #go to loc[0] start navi to loc[1]
                        print("go to {}, start navi to {}".format(self.target_location[0],self.target_location[1]))
                        # 关闭录音节点
                        self.record_switch("off")
                        self.sh.say("Now I start navigation", self.voice)
                        self.navigate_to(self.target_location[0])
                        while (True):
                            if self._navigate == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                                rospy.sleep(5)
                                self.image_process("person")
                                break
                        while (True):
                            if self._person == FINISH:
                                rospy.sleep(3)
                                os.system("rosnode kill /person_detection")
                                string = "Wow i have found you, please follow me to the " + self.target_location[1]
                                self.sh.say (string, self.voice)
                                self.navigate_to(self.target_location[1])
                                break
                        self.navigate_to("operator")
                        while (True):
                            if self._navigate == FINISH:
                                self.init_params()
                                self.sh.say("okay i am ready for your next command", self.voice)
                                break
                        self.init_params()
            else:
                if self.target_people  and self.target_location:
                    self.task_type = "people"
                    print("task type is {}".format(self.task_type))
                    if 'me' in output:
                        #go to target_loc[0], recognize person by feature and go back to answer
                        if 'gender' in output:
                            print("go to {}, recognize the gender of the person and go back to answer".format(self.target_location[0]))
                            # 关闭录音节点
                            self.record_switch("off")
                            self.sh.say("Now I start navigation", self.voice)
                            self.navigate_to(self.target_location[0])
                            while (True):
                                if self._navigate == FINISH:
                                    rospy.loginfo("Start finding person")
                                    self.sh.say("Now start finding person, please wait", self.voice)
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                                    rospy.sleep(5)
                                    self.image_process("person")
                                    break
                            while (True):
                                if self._person == FINISH:
                                    rospy.loginfo("Start gender recognition")
                                    rospy.sleep(3)
                                    os.system("rosnode kill /person_detection")
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core gender_recognition.py'")
                                    rospy.sleep(5)
                                    self.image_process("gender")
                                    break
                            while (True):
                                if self._mission == FINISH:
                                    rospy.loginfo("Back to the start point")
                                    # @TODO
                                    # 此处修改为设定好的start point
                                    rospy.sleep(12)
                                    self.navigate_to("operator")
                                    break
                            while (True):
                                if self._navigate==FINISH:
                                    rospy.sleep(5)
                                    string = self._person_gender
                                    print(string)
                                    self.sh.say(string,self.voice)
                                    break
                            self.init_params()
                            
                        if 'pose' in output or 'post' in output:
                            print("go to {}, recognize the pose of the person and go back to answer".format(self.target_location[0]))
                            # 关闭录音节点
                            self.record_switch("off")
                            self.sh.say("Now I start navigation", self.voice)
                            self.navigate_to(self.target_location[0])
                            while (True):
                                if self._navigate == FINISH:
                                    rospy.loginfo("Start finding person")
                                    self.sh.say("Now start finding person, please wait", self.voice)
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                                    rospy.sleep(5)
                                    self.image_process("person")
                                    break
                            while (True):
                                if self._person == FINISH:
                                    rospy.loginfo("Start pose recognition")
                                    rospy.sleep(3)
                                    os.system("rosnode kill /person_detection")
                                    self.sh.say("Now start recognize the gesture of people Please wait", self.voice)
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core pose_detection.py'")
                                    rospy.sleep(5)
                                    self.image_process("gesture")
                                    break
                            while (True):
                                if self._mission == FINISH:
                                    rospy.loginfo("Back to the start point")
                                    rospy.sleep(20)
                                    self.navigate_to("operator")
                                    break
                            while (True):
                                if self._navigate==FINISH:
                                    rospy.sleep(5)
                                    print(self._person_pose)
                                    self.sh.say("the person in the {} is {}".format(self.target_location[0], self._person_pose.data),self.voice)
                                    break
                            self.init_params()
                        if 'name' in output:
                            print("go to {}, ask the name of the person and go back to answer".format(self.target_location[0]))
                            # 关闭录音节点
                            self.record_switch("off")
                            while (True):
                                if self._navigate == FINISH:
                                    rospy.loginfo("Start finding person")
                                    self.sh.say("Now start finding person, please wait", self.voice)
                                    os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                                    rospy.sleep(5)
                                    self.image_process("person")
                                    break
                            while (True):
                                if self._person == FINISH:
                                    rospy.sleep(3)
                                    os.system("rosnode kill /person_detection")
                                    self.sh.say("Hello my name is Jack, Please tell me your name", self.voice)
                                    # @TODO not finished
                                    break
                            self.init_params()
                    else:
                        #go to loc[0] and talk  #talk 写成函数,输入是output #find_person_by_feature(feature)
                        print("go to {}, find the person by feature {} and talk".format(self.target_location[0],self.target_gender+self.target_gesture+self.target_pose))
                        # 关闭录音节点
                        self.record_switch("off")
                        self.sh.say("Now I start navigation", self.voice)
                        gesture = ''
                        if self.target_gesture:
                            gesture = self.target_gesture[0]
                        if self.target_pose:
                            gesture = self.target_pose[0]
                            
                        self.navigate_to(self.target_location[0])
                        while (True):
                            if self._navigate == FINISH:
                                rospy.loginfo("Start finding person")
                                self.sh.say("Now start finding person, please wait", self.voice)
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core person_detection.py'") 
                                rospy.sleep(5)
                                self.image_process("person")
                                break
                        while (True):
                            # 如果没有找到指定特征的人，则将_person置为UNSTART，然后重新开始找人
                            if self._person == UNSTART:
                                rospy.loginfo("Restart find person")
                                self.image_process("person")
                            # 目前只考虑pose和gender， 姓名暂时无法完成
                            # 如果当前任务状态(_mission)为UNSTART, 表明还没有开启gesture识别的节点
                            # 开启节点，并且发布消息进行识别
                            if self._person == FINISH and (self.target_gesture or self.target_pose) and self._mission == UNSTART:
                                rospy.loginfo("Start pose detection node")
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core pose_detection.py'")
                                rospy.sleep(6)
                                rospy.loginfo("Send message")
                                self.image_process("gesture", gesture)
                                continue

                            if self._person == FINISH and self.target_gender and self._mission == UNSTART:
                                rospy.loginfo("Start gender recognition node")
                                os.system("gnome-terminal -x bash -c 'rosrun kamerider_image_core gender_recognition.py'")
                                rospy.sleep(6)
                                rospy.loginfo("send message")
                                self.image_process("gender", self.target_gender[0])              
                                continue
            
                            # 如果任务状态(_mission)为PROCESS， 表明节点已经开启，但是没有找到指定的gesture
                            # 此时如果_person为FINISH， 表明已经找到了下一个人，
                            if self._person == FINISH and (self.target_gesture or self.target_pose) and self._mission == FAILED:
                                rospy.loginfo("ReStart gesture recognition")
                                self.image_process("gesture", gesture)

                            if self._person == FINISH and self.target_gender and self._mission == FAILED:
                                rospy.loginfo("ReStart gender recognition")
                                self.image_process("gender", self.target_gender[0])
                            
                            # 如果完成了指定gesture的识别，则进入说话环节
                            # 按照要求讲笑话，介绍自己，介绍队伍，或者回答问题
                            if self._mission == FINISH:
                                rospy.loginfo("Target gesture detected!")
                                # 启动录音节点，准备回答问题
                                self.record_switch("on")
                                string = "i have found the person {}".format(self.target_gender+self.target_gesture+self.target_pose)
                                self.sh.say(string)
                                if self.target_detail:
                                    answer_question(self.sh, self.voice, self.target_detail)
                                else:
                                    self.sh.say("Hello i am jack, please ask me the question or speak to me", self.voice)
                else:
                    if (self.target_object or 'object' in output) and self.target_location:
                        self.task_type = "object"
                        print("task type is {}".format(self.task_type))
                        if 'find' in output or 'locate' in output:
                            #find obj
                            print("find {}".format(self.target_object))
                        if 'many' in output:
                            #find the num of obj
                            print("find how many {} are there at {}".format(self.target_object,self.target_location))
                        if 'what' in output:
                            #find the biggest obj 
                            print("find the biggest {} at {}".format(self.target_object,self.target_location))
                    else:
                        self.task_type = "Q&A"
                        print("task type is {}".format(self.task_type))
                        # Uncomment this to answer questions
                        answer_question(self.sh, self.voice, output)
if __name__ == '__main__':
    rospy.init_node("gpsr_speech_control", anonymous=True)
    ctrl = gpsr_speech_control()
    rospy.spin()                           





                

            

        







                
        




            





                    

        
                




        
    
        
    
