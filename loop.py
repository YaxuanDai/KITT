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
import matplotlib.pyplot as plt
import time
import cv2

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math
import os

from car import Car
from infrad import Infrad
from lane_lines import *

krate_sum=[ 0 for i in range(10) ]

def forward(car):
    car.set_speed(60, 60)
    time.sleep(0.1)
    car.set_speed(0, 0)

def find_left(car):
    car.set_speed(-100, 100)
    time.sleep(0.15)
    car.set_speed(50, 50)

def find_right(car):
    car.set_speed(100, -100)
    time.sleep(0.15)
    car.set_speed(50, 50)

def rush_left(car):
    car.set_speed(-200, 200)
    time.sleep(0.1)
    car.set_speed(50, 50)

def rush_right(car):
    car.set_speed(-200, 200)
    time.sleep(0.2)
    car.set_speed(50, 50)
    
def set_slow(car):
    car.set_speed(-80, -80)
    time.sleep(0.25)
    car.set_speed(-160, 160)
    time.sleep(0.2)
    car.set_speed(50, 50)

def krate(line):
    # compute the sign of the slop of the line
    rate = (line[0] - line[2]) / (line[1] - line[3]) 
    return round(rate, 4)

def kratesum(lines):
    global krate_sum
    rsum = krate(lines[0]) + krate(lines[1])
    del krate_sum[0]
    krate_sum.append(rsum)
    result=np.fft.fft(krate_sum)
    return(result)

def kratesum(lines):
    return krate(lines[0]) + krate(lines[1])

def stage_control(lines, car):
    if lines!=None and len(lines)==2:
        k = kratesum(lines)
    else:
        return -1, -1

    if abs(k) <= 4:
        forward(car)
        v1, v2 = 60, 60
    elif k < -4:
        find_right(car)
        v1, v2 = 60, -60
    elif k > 4:
        find_left(car)
        v1, v2 = -60, 60
    # car.set_speed(v1, v2)
    return v1, v2

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
    
if __name__ == '__main__':
  car = Car()
  inf = Infrad()
  v1, v2 = 60, 60
  car.set_speed(v1, v2)
  try:
    while True:
      left, right, nl, nr = inf.detect()
      # print(left, right)
      left_ans = True if left else False
      right_ans = True if right else False
      new_left = True if nl else False
      new_right = True if nr else False
      print(str(left_ans) + ", " + str(right_ans) + ", " + str(new_left) + ', ' + str(new_right))
      if not left_ans and right_ans and new_left and new_right:
        find_right(car)
      elif not right_ans and left_ans and new_left and new_right:
        find_left(car)
      elif not new_left and new_right:
        rush_left(car)
      elif not new_right and new_left:
        rush_right(car)
      elif not new_left and not new_right:
        set_slow(car)
  except KeyboardInterrupt:
    GPIO.cleanup()
"""
    v1, v2 = 0, 0
    rawCapture = picamera.array.PiRGBArray(camera, size = im_size)
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        image = frame.array
        image = image.reshape((640, 480, 3))
        rawCapture.truncate(0)
        lines = stage_detect(image)
        v1, v2 = stage_control(lines, car)
        # print(v1, v2)
        if lines != None:
            left, right = lines[0], lines[1]
            if left[0] > right[0]:
                left[0], right[0] = right[0], left[0]
            print(left[0]/320, right[0]/320, v1, v2)
        else:
            print(-1, -1)
"""