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
from car import selfcontrol
from lane_lines import *

def krate(lines):
    # compute the sign of the slop of the line
    s = (lines[0] - lines[2]) * (lines[1] - lines[3])
    return(np.sign(s))

def kratesum(lines):
    rsum = krate(lines[0]) + krate(lines[1])
    return(rsum)

def change_speed(lines):
    try:
        if abs(kratesum(lines)) <= 0.5:
            v1 = 40
            v2 = 40       
        if kratesum(lines) < -0.5:
            v1 = 120
            v2 = -120
        if kratesum(lines) > 0.5:
            v1 = -120 
            v2 = 120
    except:
        v1 = v2 = 100
    return v1, v2

def forward(car):
    car.set_speed(60, 60)
    time.sleep(3)
    car.set_speed(0, 0)

def find_left(car):
    car.set_speed(60, -60)
    time.sleep(1)
    car.set_speed(0, 0)

def find_left(car):
    car.set_speed(-60, 60)
    time.sleep(1)
    car.set_speed(0, 0)
    
if __name__ == '__main__':
    car = Car()
    camera = picamera.PiCamera()
    camera.resolution = (640, 480)
    camera.start_preview()
    # Camera warm-up time
    time.sleep(2)
    v1, v2 = 0, 0
    while 1:
        image = np.empty((640 * 480 * 3,), dtype=np.uint8)
        camera.capture(image, 'bgr')
        image = image.reshape((640, 480, 3))
        # TODD: Detect
        try:
            lines = selfcontrol(image)
            v1, v2 = change_speed(lines)
            print(kratesum(lines), v1, v2)
        except:
            v1, v2 = -100, -100
            print(v1, v2, "Find")
        # TODO: ROS 
        car.set_speed(v1, v2)