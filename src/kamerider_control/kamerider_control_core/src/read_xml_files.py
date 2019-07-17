#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
Date: 2018/03/02
Author: Xu Yucheng 
Abstract： 读取官方发布的CmdGen软件中使用的各种命令文件，总结出可能的命令模式
'''
import os
import sys
import xml
import xml.etree.ElementTree as et
import time

def read_gesture_file(path='/home/kamerider/kamerider_GPSR/src/kamerider_control/kamerider_control_core/CommonFiles/Gestures.xml'):
    xml_file = et.parse(path)
    root = xml_file.getroot()
    gesture_xml = []  
    gestures = root.findall('gesture')
    for ges in gestures:
        gesture_xml.append(ges.attrib['name'])
    return gesture_xml

def read_location_xml(path='/home/kamerider/kamerider_GPSR/src/kamerider_control/kamerider_control_core/CommonFiles/Locations.xml'):
    location_xml = {}    
    xml_file = et.parse(path)
    root = xml_file.getroot()
    rooms = root.getchildren()
    for room in rooms:
        location_xml[room.attrib['name']]=[]
        locs = room.getchildren()
        for loc in locs:
            location_xml[room.attrib['name']].append(loc.attrib['name'])
    return location_xml

# def read_names_xml(path='/home/kamerider/kamerider_GPSR/src/kamerider_control/kamerider_control_core/CommonFiles/Names.xml'):
#     name_xml = {}
#     xml_file = et.parse(path)
#     root = xml_file.getroot()
#     names = root.getchildren()
#     for name in names:
#         name_xml[name.text] = name.attrib['gender']
#     return name_xml

def read_object_xml(path='/home/kamerider/kamerider_GPSR/src/kamerider_control/kamerider_control_core/CommonFiles/Objects.xml'):
    object_xml = {}
    xml_file = et.parse(path)
    root = xml_file.getroot()
    categorys = root.getchildren()
    for category in categorys:
        objects = category.getchildren()
        for obj in objects:
            object_xml[obj.attrib['name']] = dict()
            object_xml[obj.attrib['name']]['location'] = category.attrib['defaultLocation']
            object_xml[obj.attrib['name']]['room'] = category.attrib['room']
    return object_xml

    

def main():
    gestures = read_gesture_file()
    locations = read_location_xml()
    # names = read_names_xml()
    objects = read_object_xml()
    # print (objects)
    # print (locations)
    # print (gestures)
    # print (locations.keys())
    # print (names.keys())

    return gestures, locations, objects

if __name__ == '__main__':
    main()
