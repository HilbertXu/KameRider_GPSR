#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2018/03/02
Author: Xu Yucheng
Abstract： 订阅输出Pocketsphinx识别结果的话题，然后根据输出的结果来控制机器人
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
import sys
import time
import wave
import datetime
import pyaudio
from sound_play.libsoundplay import SoundClient
from read_xml_files import main as read_main
from play_signal_sound import play_signal_sound

"""
    param 1: SoundHandle
    param 2: recognition output
    param 3: voice you want to use
    usage(from new_gpsr_control.py): answer_question(self.sh, self.voice, output) 
"""
def answer_question(sh, voice, output, flag=None):
    if "joke" in output:
        """
            给他们讲个冷笑话...
        """
        sh.say("Now i will tell a joke")
        sh.say("Trump's nothing like Hitler. There's no way he could write a book")
        sh.say("haaahaahahaah")
        flag = True
    if "yourself" in output:
        sh.say("Now i will say something about my self")
        sh.say("My name is Jack, i come from Nan kai university Tian Jin province.")
        sh.say("Nice to meet you")
        flag = True
    if "affiliation" in output or "relation" in output:
        sh.say("Our team is affiliated to Nan Kai university")
        flag = True
    
    #################################################################
    ###################      NEW GPSR QUESTIONS     #################
    #################################################################
    if "program" in output or "programming" in output or "programmer" in output:
        if "who" in output:
            if "c" in output:
                sh.say("I heard the question", voice)
                sh.say("Who invented the C programming language", voice)
                sh.say("The answer is Ken Thompson and Dennis Ritchie", voice)
                sh.say("Okay i am ready for your next question", voice)
            if "python" in output:
                sh.say("I heard the question", voice)
                sh.say("Who created the Python Programming Language", voice)
                sh.say("The answer is Python was invented by Guido van Rossum", voice)
                sh.say("Okay i am ready for your next question", voice)
            if "first" in output:
                sh.say("I heard the question", voice)
                sh.say("Who is considered to be the first computer programmer", voice)
                sh.say("The answer is Ada Lovelace", voice)
                sh.say("Okay i am ready for your next question", voice)
        if "when" in output:
            if "c" in output:
                sh.say("I heard the question", voice)
                sh.say("When was the C programming language invented", voice)
                sh.say("The answer is C was developed after B in 1972 at Bell Labs", voice)
                sh.say("Okay i am ready for your next question", voice)
            if "b" in output or "big" in output:
                sh.say("I heard the question", voice)
                sh.say("When was the B programming language invented", voice)
                sh.say("The answer is B was developed circa 1969 at Bell Labs", voice)
                sh.say("Okay i am ready for your next question", voice)
        if "which" in output:
            sh.say("I heard the question", voice)
            sh.say("Which program do Jedi use to open PDF files", voice)
            sh.say("The answer is Adobe Wan Kenobi", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "computer" in output:
        if "but" in output or "bug" in output:
            if "where" in output:
                sh.say("I heard the question", voice)
                sh.say("Where does the term computer bug come from", voice)
                sh.say("The answer is From a moth trapped in a relay", voice)
                sh.say("Okay i am ready for your next question", voice)
            if "first" in output:
                sh.say("I heard the question", voice)
                #bug识别为back bag
                sh.say("What was the first computer bug", voice)
                sh.say("The answer is The first actual computer bug was a dead moth stuck in a Harvard Mark II", voice)
                sh.say("Okay i am ready for your next question", voice)
        if "test" in output or "pass" in output:
            sh.say("I heard the question", voice)
            sh.say("What was the first computer in pass the Turing test", voice)
            sh.say("The answer is Some people think it was IBM Watson, but it was Eugene, a computer designed at England's University of Reading", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "compile" in output or "compiler" in output:
        sh.say("I heard the question", voice)
        sh.say("Who invented the first compiler", voice)
        sh.say("The answer is Grace Brewster Murray Hopper invented it", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "platform" in output:
        if "open" in output:
            sh.say("I heard the question", voice)
            sh.say("Which robot is used in the Open Platform League", voice)
            sh.say("The answer is There is no standard defined for OPL", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "domestic" in output:
            sh.say("I heard the question", voice)
            sh.say("Which robot is used in the Domestic Standard Platform League", voice)
            sh.say("The answer is The Toyota Human Support Robot", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "social" in output:
            sh.say("I heard the question", voice)
            sh.say("Which robot is used in the Social Standard Platform League", voice)
            sh.say("The answer is The SoftBank Robotics Pepper", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "name" in output and "team" in output:
        sh.say("I heard the question", voice)
        sh.say("What's the name of your team", voice)
        sh.say("The answer is kamerider", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "time" in output and len(output) < 6:
        curr_time = str(time.strftime('%H:%M:%S'))
        sh.say("I heard the question", voice)
        sh.say("What time is it", voice)
        sh.say("The answer is "+curr_time, voice)
        sh.say("Okay i am ready for your next question", voice)
    if "day" in output or "today" in output:
        today = datetime.date.today()
        today = today.strftime("%Y-%m-%d")
        year = today[0]
        month = today
        sh.say("I heard the question", voice)
        sh.say("What day is today", voice)
        sh.say("The answer is "+today, voice)
        sh.say("Okay i am ready for your next question", voice)
    if "dream" in output or "dreams" in output:
        sh.say("I heard the question", voice)
        sh.say("Do you have dreams", voice)
        sh.say("The answer is I dream of Electric Sheep", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "city" in output and "next" in output:
        sh.say("I heard the question", voice)
        sh.say("In which city will next year's RoboCup be hosted", voice)
        sh.say("The answer is It hasn't been announced yet", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "canada" in output:
        if "origin" in output:
            sh.say("I heard the question", voice)
            sh.say("What is the origin of the name Canada", voice)
            sh.say("The answer is The name Canada comes from the Iroquois word Kanata, meaning village or settlement", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "capital" in output:
            sh.say("I heard the question", voice)
            sh.say("What is the capital of Canada", voice)
            sh.say("The answer is The capital of Canada is Ottawa", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "national" in output:
            sh.say("I heard the question", voice)
            sh.say("What is the national anthem of Canada", voice)
            sh.say("The answer is O Canada", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "handsome" in output or "person" in output:
            sh.say("I heard the question", voice)
            sh.say("Who's the most handsome person in Canada", voice)
            sh.say("The answer is that Justin Trudeau is very handsome", voice)
            sh.say("Okay i am ready for your next question", voice)
        if ("time" in output or "many" in output) and len(output) < 9:
            sh.say("I heard the question", voice)
            sh.say("How many time zones are there in Canada", voice)
            sh.say("The answer is Canada spans almost 10 million square km and comprises 6 time zones", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "usa" in output or "year" in output:
            if "first" in output:
                sh.say("I heard the question", voice)
                sh.say("In which year was Canada invaded by the USA for the first time", voice)
                sh.say("The answer is The first time that the USA invaded Canada was in 1775", voice)
                sh.say("Okay i am ready for your next question", voice)
            if "second" in output:
                sh.say("I heard the question", voice)
                sh.say("What year was Canada invaded by the USA for the second time", voice)
                sh.say("The answer is The USA invaded Canada a second time in 1812", voice)
                sh.say("Okay i am ready for your next question", voice)
        if "why" in output:
            sh.say("I heard the question", voice)
            sh.say("Why is Canada named Canada", voice)
            sh.say("The answer is French explorers misunderstood the local native word Kanata, which means village", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "desert" in output:
            sh.say("I heard the question", voice)
            #canada有？
            sh.say("Where is Canada's only desert", voice)
            sh.say("The answer is Canada's only desert is British Columbia", voice)
            sh.say("Okay i am ready for your next question", voice)

    if "world" in output or "word" in output or "worlds" in output or "words" in output or "world's" in output:
        if "longest" in output or "street" in output:
            sh.say("I heard the question", voice)
            sh.say("What's the longest street in the world", voice)
            sh.say("The answer is Yonge Street in Ontario is the longest street in the world", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "largest" in output or "coin" in output:
            sh.say("I heard the question", voice)
            sh.say("What is the world's largest coin", voice)
            sh.say("The answer is The Big Nickel in Sudbury, Ontario. It is nine meters in diameter", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "street" in output and "how" in output:
        sh.say("I heard the question", voice)
        sh.say("How long is Yonge Street in Ontario", voice)
        sh.say("The answer is Yonge street is almost 2,000 km, starting at Lake Ontario, and running north to the Minnesota border", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "london" in output and "name" in output:
        sh.say("I heard the question", voice)
        sh.say("What's the name of the bear cub exported from Canada to the London Zoo in 1915", voice)
        sh.say("The answer is The bear cub was named Winnipeg. It inspired the stories of Winnie-the-Pooh", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "smartphone" in output or "smart" in output or "phone" in output:
        sh.say("I heard the question", voice)
        sh.say("Where was the Blackberry Smartphone developed", voice)
        sh.say("The answer is It was developed in Ontario, at Research In Motion's Waterloo offices", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "country" in output:
        if "record" in output or "gold" in output:
            sh.say("I heard the question", voice)
            sh.say("What country holds the record for the most gold medals at the Winter Olympics", voice)
            sh.say("The answer is Canada does! With 14 Golds at the 2010 Vancouver Winter Olympics", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "who" in output and "coined" in output:
        if "term" in output:
            sh.say("I heard the question", voice)
            sh.say("Who coined the term Beatlemenia", voice)
            sh.say("Sandy Gardiner, a journilist of the Ottawa Journal", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "mounted" in output or "police" in output:
        if "royal" in output:
            sh.say("I heard the question", voice)
            sh.say("When was The Royal Canadian Mounted Police formed", voice)
            sh.say("The answer is In 1920, when The Mounted Police merged with the Dominion Police", voice)
            sh.say("Okay i am ready for your next question", voice)
        else:
            sh.say("I heard the question", voice)
            sh.say("When was The Mounted Police formed", voice)
            sh.say("The answer is The Mounted Police was formed in 1873", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "how" in output and "big" in output:
        if "RCMP" in output:
            sh.say("I heard the question", voice)
            sh.say("How big is the RCMP", voice)
            sh.say("The answer is Today, the RCMP has close to 30,000 members", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "canda" in output or "desert" in output:
            sh.say("I heard the question", voice)
            sh.say("How big is Canada's only desert", voice)
            sh.say("The answer is The British Columbia desert is only 15 miles long", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "montreal" in output or "else" in output:
        sh.say("I heard the question", voice)
        sh.say("What else is Montreal called", voice)
        sh.say("The answer is Montreal is often called the City of Saints or the City of a Hundred Bell Towers", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "hotel" in output:
        if "where" in output:
            sh.say("I heard the question", voice)
            sh.say("Where is The Hotel de Glace located", voice)
            sh.say("The answer is The Hotel de Glace is in Quebec", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "how" in output or "many" in output:
            if "ice" in output or "see" in output:
                sh.say("I heard the question", voice)
                sh.say("How many tons of ice are required to build The Hotel de Glace", voice)
                sh.say("The answer is The Hotel de Glace requires about 400 tons of ice", voice)
                sh.say("Okay i am ready for your next question", voice)
            if "snow" in output:
                sh.say("I heard the question", voice)
                sh.say("How many tons of snow are required to build The Hotel de Glace", voice)
                sh.say("The answer is Every year, 12000 tons of snow are used for The Hotel de Glace", voice)
                sh.say("Okay i am ready for your next question", voice)
        if "can" in output or "visit" in output:
            sh.say("I heard the question", voice)
            sh.say("Can I visit the Hotel de Glace in summer", voice)
            sh.say("The answer is No. Every summer it melts away, only to be rebuilt the following winter", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "name" in output:
        if "famous" in output:
            if "male" in output:
                sh.say("I heard the question", voice)
                sh.say("Name 3 famous male Canadians", voice)
                sh.say("The answer is Leonard Cohen, Keanu Reeves, and Jim Carrey", voice)
                sh.say("Okay i am ready for your next question", voice)
            if "female" in output:
                sh.say("I heard the question", voice)
                sh.say("Name 3 famous female Canadians", voice)
                sh.say("The answer is Celine Dion, Pamela Anderson, and Avril Lavigne", voice)
                sh.say("Okay i am ready for your next question", voice)
        if "robots" in output or "all" in output or "robot" in output:
            sh.say("I heard the question", voice)
            sh.say("Name all of the robots on Mars", voice)
            sh.say("The answer is There are four robots on Mars: Sojourner, Spirit, Opportunity, and Curiosity. Three more crashed on landing", voice)
            sh.say("Okay i am ready for your next question", voice)
    if ("what" in output or "what's" in output) and "origin" in output:
        if "comic" in output or "font" in output:
            sh.say("I heard the question", voice)
            sh.say("What's the origin of the Comic Sans font", voice)
            sh.say("The answer is Comic Sans is based on Dave Gibbons' lettering in the Watchmen comic books", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "nanobot" in output or "nano" in output or "another" in output:
        if "what" in output:
            sh.say("I heard the question", voice)
            sh.say("what is a nanobot", voice)
            sh.say("The answer is The Hotel de Glace is in Quebec", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "how" in output:
            sh.say("I heard the question", voice)
            sh.say("How small can a nanobot be", voice)
            sh.say("The answer is A nanobot can be less than one-thousandth of a millimeter", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "why" in output and "award" in output:
        sh.say("I heard the question", voice)
        sh.say("Why wasn't Tron nominated for an award by The Motion Picture Academy", voice)
        sh.say("The answer is The Academy thought that Tron cheated by using computers", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "hard" in output and "disk" in output:
        if "which" in output:
            sh.say("I heard the question", voice)
            sh.say("Which was the first computer with a hard disk drive", voice)
            sh.say("The answer is The IBM 305 RAMAC", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "when" in output:
            sh.say("I heard the question", voice)
            sh.say("When was the first computer with a hard disk drive launched", voice)
            sh.say("The answer is The IBM 305 RAMAC was launched in 1956", voice)
            sh.say("Okay i am ready for your next question", voice)
        if "how" in output or "big" in output:
            sh.say("I heard the question", voice)
            sh.say("How big was the first hard disk drive", voice)
            sh.say("The answer is The IBM 305 RAMAC hard disk weighed over a ton and stored 5 MB of data", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "what" in output and ("stand" in output or "stands" in output):
        sh.say("I heard the question", voice)
        sh.say("What does CAPTCHA stands for", voice)
        sh.say("The answer is CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "first" in output and "android" in output:
        sh.say("I heard the question", voice)
        sh.say("Who is the world's first android", voice)
        sh.say("The answer is Professor Kevin Warwick uses chips in his arm to operate doors, a robotic hand, and a wheelchair", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "mechanical" in output:
        sh.say("I heard the question", voice)
        sh.say("What is a Mechanical Knight", voice)
        sh.say("The answer is A robot sketch made by Leonardo DaVinci", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "paradox" in output or "state" in output:
        sh.say("I heard the question", voice)
        sh.say("What does Moravec's paradox state", voice)
        sh.say("The answer is Moravec's paradox states that a computer can crunch numbers like Bernoulli, but lacks a toddler's motor skills", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "knowledge" in output or "engineering" in output:
        sh.say("I heard the question", voice)
        sh.say("What is the AI knowledge engineering bottleneck", voice)
        sh.say("The answer is It is when you need to load an AI with enough knowledge to start learning", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "why" in output:
        if "worried" in output or "impact" in output:
            sh.say("I heard the question", voice)
            sh.say("Why is Elon Musk is worried about AI's impact on humanity", voice)
            sh.say("The answer is I don't know. He should worry more about the people's impact on humanity", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "you" in output or "do" in output:
        if "threat" in output or "humanity" in output:
            sh.say("I heard the question", voice)
            sh.say("Do you think robots are a threat to humanity", voice)
            sh.say("The answer is No Humans are the real threat to humanity", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "what" in output and "chatbot" in output:
        sh.say("I heard the question", voice)
        sh.say("What is a chatbot", voice)
        sh.say("The answer is A chatbot is an A.I. you put in customer service to avoid paying salaries", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "car" in output and "safe" in output:
        sh.say("I heard the question", voice)
        sh.say("Are self-driving cars safe", voice)
        sh.say("The answer is Yes. Car accidents are product of human misconduct", voice)
        sh.say("Okay i am ready for your next question", voice)
    if "robot" in output:
        if "mark" in output or "is" in output:
            sh.say("I heard the question", voice)
            sh.say("Is Mark Zuckerberg a robot", voice)
            sh.say("The answer is Sure. I've never seen him drink water", voice)
            sh.say("Okay i am ready for your next question", voice)
    if "apple" in output:
        sh.say("I heard the question", voice)
        sh.say("Who is the inventor of the Apple I microcomputer", voice)
        sh.say("The answer My lord and master Steve Wozniak", voice)
        sh.say("Okay i am ready for your next question", voice)
################################################################################
    
