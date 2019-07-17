#! /usr/bin/env python
# -*- coding: utf-8 -*-

import pyaudio
import wave
import os
import sys

def play_signal_sound():
    question_start_signal = '/home/kamerider/kamerider_GPSR/src/kamerider_speech/kamerider_speech_core/sounds/question_start_signal.wav'
    chunk = 1024
    # 打开 .wav 音频文件
    f = wave.open(question_start_signal, 'rb')
    # 初始化pyaudio
    p = pyaudio.PyAudio()
    # 打开一个stream
    stream = p.open(
        format = p.get_format_from_width(f.getsampwidth()),
        channels = f.getnchannels(),
        rate = f.getframerate(),
        output = True
    )
    # 读取音频文件中的数据
    data = f.readframes(chunk)

    # 播放音频文件
    while data != '':
        stream.write(data)
        data = f.readframes(chunk)
    
    # 终止stream
    stream.stop_stream()
    stream.close()
    # 关闭pyaudio
    p.terminate()
