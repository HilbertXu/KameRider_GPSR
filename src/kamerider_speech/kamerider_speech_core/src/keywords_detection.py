#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2018/12/17
Author: Xu Yucheng 
Abstract： 使用pocketshpinx识别语句中的关键词，并将结果发布到ros话题中
'''

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
from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *

class keywords_detection(object):
    """ Class to spot keyword in GPSR project"""
    def __init__(self):
        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("kws_data", String, queue_size=10)
        # Initalizing publisher for continuous ASR
        self.continuous_pub_ = rospy.Publisher(
            "jsgf_audio", String, queue_size=10)

        # initialize node
        rospy.init_node("kws_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        # Params

        # File containing language model
        _hmm_param = "~hmm"
        # Dictionary
        _dict_param = "~dict"
        # List of keywords to detect
        _kws_param = "~kws"
        """Not necessary to provide the next two if _kws_param is provided
        Single word which needs to be detected
        """
        _keyphrase_param = "~keyphrase"
        # Threshold frequency of above mentioned word
        _threshold_param = "~threshold"
        # Option for continuous
        self._option_param = "~option"

        # Variable to distinguish between kws list and keyphrase.
        # Default is keyword list
        self._list = True

        self.stop_output = False

        # Setting param values
        if rospy.has_param(_hmm_param):
            self.class_hmm = rospy.get_param(_hmm_param)
            if rospy.get_param(_hmm_param) == ":default":
                if os.path.isdir("/usr/local/share/pocketsphinx/model"):
                    rospy.loginfo("Loading the default acoustic model")
                    self.class_hmm = "/usr/local/share/pocketsphinx/model/en-us/en-us"
                    rospy.loginfo("Done loading the default acoustic model")
                else:
                    rospy.logerr(
                        "No language model specified. Couldn't find defaut model.")
                    return
        else:
            rospy.loginfo("Couldn't find lm argument")

        if rospy.has_param(_dict_param) and rospy.get_param(_dict_param) != ":default":
            self.lexicon = rospy.get_param(_dict_param)
        else:
            rospy.logerr(
                'No dictionary found. Please add an appropriate dictionary argument.')
            return

        if rospy.has_param(_kws_param) and rospy.get_param(_kws_param) != ":default":
            self._list = True

            self.kw_list = rospy.get_param(_kws_param)
        elif rospy.has_param(_keyphrase_param) and \
        rospy.has_param(_threshold_param) and \
        rospy.get_param(_keyphrase_param) != ":default" and \
        rospy.get_param(_threshold_param) != ":default":
            self._list = False

            self.keyphrase = rospy.get_param(_keyphrase_param)
            self.kws_threshold = rospy.get_param(_threshold_param)
        else:
            rospy.logerr(
                'kws cant run. Please add an appropriate keyword list.')
            return

        # All params satisfied. Starting recognizer
        self.start_recognizer()

    def start_recognizer(self):
        """Function to handle keyword spotting of input audio"""

        config = Decoder.default_config()
        rospy.loginfo("Pocketsphinx initialized")

        # Setting config of decoder
        config.set_string('-hmm', self.class_hmmS)
        config.set_string('-dict',self.lexicon)
        config.set_string('-dither', "no")
        config.set_string('-featparams', os.path.join(self.class_hmm, "feat.params"))

        if self._list:
            # Keyword list file for keyword searching
            config.set_string('-kws', self.kw_list)
        else:
            # In case keyphrase is provided
            config.set_string('-keyphrase', self.keyphrase)
            config.set_float('-kws_threshold', self.kws_threshold)
        
        # Set required configuration for decoder
        self.decoder = Decoder(config)

        # Start processing input audio from ros topic 
        # rosrun pocketsphinx send_audio.py
        # send_audio.py can convert audio input to rostopic 
        try:
            self.decoder = Decoder(config)
            rospy.loginfo("Decoder started successfully")
        except:
            rospy.loginfo("Failed to started Decoder!!!")
        
        # Subscriber to topic of input audio
        rospy.Subscriber("sphinx_audio", String, self.process_audio)


    def process_audio(self, data):
        # For continuous mode
        need_continuous = rospy.has_param(self._option_param)

        # Check if keyword detected
        if not self.stop_output:
            self.decoder.process_raw(data.data, False, False)
            
            if self.decoder.hyp() != None:
                rospy.loginfo([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                                for seg in self.decoder.seg()])
                rospy.loginfo("Detected keyphrase, restarting search")
                seg.word = seg.word.lower()
                self.decoder.end_utt()
                self.pub_.publish(seg.word)

                if need_continuous:
                    # Restart decoder for continuous recognize
                    print ('INSIDE')
                    self.stop_output = True
                self.decoder.start_utt()
        else:
            self.continuous_pub_.publish(data.data)

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown"""
        rospy.loginfo("Stop pocketsphinx node")
        rospy.sleep(1)
    
    if __name__ == '__main__':
        keywords_detection()