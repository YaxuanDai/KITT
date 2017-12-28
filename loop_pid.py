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

car = Car()
inf = Infrad()
ul = Ultrasound()
camera = PiCamera()

def find_left(car, GO):
    car.set_speed(-100, 100)
    time.sleep(0.15)
    if GO:
        car.set_speed(50, 50)
    else:
        car.set_speed(0, 0)

def find_right(car, GO):
    car.set_speed(100, -100)
    time.sleep(0.15)
    if GO:
        car.set_speed(50, 50)
    else:
        car.set_speed(0, 0)

def rush_left(car, GO):
    car.set_speed(-200, 200)
    time.sleep(0.1)
    if GO:
        car.set_speed(50, 50)
    else:
        car.set_speed(0, 0)

def rush_right(car, GO):
    car.set_speed(-200, 200)
    time.sleep(0.2)
    if GO:
        car.set_speed(50, 50)
    else:
        car.set_speed(0, 0)
    
def set_slow(car, GO):
    car.set_speed(-80, -80)
    time.sleep(0.25)
    car.set_speed(-160, 160)
    time.sleep(0.2)
    if GO:
        car.set_speed(50, 50)
    else:
        car.set_speed(0, 0)

def set_forward(car, GO):
    if GO:
        car.set_speed(50, 50)
    else:
        car.set_speed(0, 0)

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

def get_direction(left, right, nl, nr):
    result=0
    if(left):
        result=-1
    if(right):
        result=1
    if(nl):
        result=-2
    if(nr):
        result=2
    return result

def set_controll(correction,car,GO):
    if -1<correction<1:
        set_forward(car, GO)
    elif -2<correction<-1:
        find_right(car, GO)
    elif 2>correction>1:
        find_left(car, GO)
    elif correction<-2:
        rush_left(car, GO)
    elif correction>2:
        rush_right(car, GO)
    elif correction<-2 or correction>2:
        set_slow(car, GO)

def ros_pid(lane, car, GO):
    pid = pidcontroller.PID(1.0, 0.5, 0.1)
    target_direction = 0  # set forward
    while (True):
      current_direction = get_direction(lane)
      error = target_direction - current_direction
      correction = pid.Update(error)
      print('Setting the direction to %f' % correction)
      set_controll(correction,car,GO)

def ros(lane, car, GO):
    left, right, nl, nr = lane
    left_ans = True if left else False
    right_ans = True if right else False
    new_left = True if nl else False
    new_right = True if nr else False
    # print(str(left_ans) + ", " + str(right_ans) + ", " + str(new_left) + ', ' + str(new_right))
    if left_ans and right_ans and new_left and new_right:
        set_forward(car, GO)
    elif not left_ans and right_ans and new_left and new_right:
        find_right(car, GO)
    elif not right_ans and left_ans and new_left and new_right:
        find_left(car, GO)
    elif not new_left and new_right:
        rush_left(car, GO)
    elif not new_right and new_left:
        rush_right(car, GO)
    elif not new_left and not new_right:
        set_slow(car, GO)

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
    global STOP
    global STILL
    try:
        while True:
            left, right, nl, nr = inf.detect()
            GO = (not STOP) and (not STILL)
            ros_pid((left, right, nl, nr), car, GO)
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    global STOP
    global STILL
    STOP = False
    STILL = False
    t_LED = threading.Thread(target = LEDframe, args=() )
    t_DIS = threading.Thread(target = DISframe, args=() )
    t_ROS = threading.Thread(target = ROSframe, args=() )

    threads = [t_ROS, t_LED, t_DIS]
    v1, v2 = 60, 60
    car.set_speed(v1, v2)
    try:
        for t in threads:
            t.start()
    except KeyboardInterrupt:
        GPIO.cleanup()