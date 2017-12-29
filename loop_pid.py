#! /usr/bin/python
# coding=utf-8

import time
import select
import sys
import os
import RPi.GPIO as GPIO
import numpy as np
import picamera
import picamera.array
from picamera import PiCamera
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import time
import cv2
import math
import threading
import pidcontroller

from car import Car
from infrad import Infrad
from lane_lines import *
from detect import *
from ultrasonic import *
from encoder import *

car = Car()
inf = Infrad()
ul = Ultrasound()
camera = PiCamera()
encoder= Encoder()
encoder.command(1)
speed=0
#最近10条方向值
directions=[ 0 for i in range(10) ]
max_speed=200

def stage_detect(image_in):
    image = filter_colors(image_in)
    gray = grayscale(image)
    blur_gray = gaussian_blur(gray, kernel_size)
    edges = canny(blur_gray, low_threshold, high_threshold)

    imshape = image.shape
    vertices = np.array([[\
        ((imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0]),\
        ((imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height),\
        (imshape[1] - (imshape[1] * (1 - trap_top_width)) // 2, imshape[0] - imshape[0] * trap_height),\
        (imshape[1] - (imshape[1] * (1 - trap_bottom_width)) // 2, imshape[0])]]\
        , dtype=np.int32)
    masked_edges = region_of_interest(edges, vertices)

    img = masked_edges
    min_line_len = min_line_length
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    if lines is None:
        return None
    line_img = np.zeros((*img.shape, 3), dtype=np.uint8)  # 3-channel RGB image
    newlines = draw_lines(line_img, lines)

    for line in newlines:
        if line[1] < line[3]:
            line[0], line[1], line[2], line[3] = line[2], line[3], line[0], line[1]
    if newlines[0][0] > newlines[1][0]:
        newlines[0], newlines[1] = newlines[1], newlines[0]
    return(newlines)

### PID Controller functions start

def get_direction(left, right, nl, nr):
    global directions
    result=0
    if(left):
        result=-1
    if(right):
        result=1
    if(nl):
        result=-2
    if(nr):
        result=2
    directions.pop(0)
    directions.extend(result)
    return result

#取得小车运动稳定性,该方法是pid控制速度关键
def get_instability(speed_left,speed_right):
    global directions
    rate=np.std(directions)
    speed=(speed_left+speed_right)/2
    if(speed>0):
        instability=rate/speed
    else:
        instability=0
    return instability

def set_speed(correction,car):
    global speed,max_speed
    speed=correction
    if speed>max_speed:
        speed=max_speed
    car.set_speed(speed, speed)

def speed_pid(car):
    pid = pidcontroller.PID(1.0, 0.5, 0.1)
    target_instability = 0
    while (True):
      current_instability = get_instability()
      error = target_instability - current_instability
      correction = pid.Update(error)
      print('Setting the speed to %f' % correction)
      #set_controll(correction,car,GO)
      set_speed(correction,car)

def set_speed_different(correction,car):
    global speed
    speed_left=speed+correction/2
    speed_right=speed-correction/2
    car.set_speed(speed_left,speed_right)

def ros_pid(lane, car):
    pid = pidcontroller.PID(1.0, 0.5, 0.1)
    target_direction = 0  # set forward
    while (True):
      current_direction = get_direction(lane)
      error = target_direction - current_direction
      correction = pid.Update(error)
      print('Setting the direction to %f' % correction)
      #set_controll(correction,car,GO)
      set_speed_different(correction,car)

### PID Controller functions end

def SPEEDframe():
    global encoder
    try:
        while True:
            left_speed, right_speed = encoder.get_speed()
            speed_pid((left_speed, right_speed), car)
    except KeyboardInterrupt:
        GPIO.cleanup()

def LEDframe():
    global STOP
    rawCapture = picamera.array.PiRGBArray(camera)
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        image = frame.array
        print(image.shape)
        image = image.reshape((640, 480, 3))
        rawCapture.truncate(0)
        light = LED_detect(image)
        if light==2:
            STOP = True
        else:
            STOP = False
        print("light: ", light)

def DISframe():
    global STILL
    try:
        while True:
            dis = round(ul.get_distance(), 2)
            # print("dis: ", dis)
            if dis < 20:
                STILL = True
            else:
                STILL = False
    except KeyboardInterrupt:
        GPIO.cleanup()
def ROSframe():
    try:
        while True:
            left, right, nl, nr = inf.detect()
            ros_pid((left, right, nl, nr), car)
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    global STOP
    global STILL
    global speed,max_speed
    STOP = False
    STILL = False
    t_LED = threading.Thread(target = LEDframe, args=() )
    t_DIS = threading.Thread(target = DISframe, args=() )
    t_ROS = threading.Thread(target = ROSframe, args=() )
    t_SPEED = threading.Thread(target = SPEEDframe, args=() )

    threads = [t_ROS, t_LED, t_DIS]
    speed=max_speed
    v1, v2 = speed, speed
    car.set_speed(v1, v2)
    try:
        for t in threads:
            t.start()
    except KeyboardInterrupt:
        GPIO.cleanup()