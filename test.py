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

def krate(line):
    # compute the sign of the slop of the line
    rate = (line[0] - line[2]) / (line[1] - line[3]) 
    return round(rate, 4)

def kratesum(lines):
    rsum = krate(lines[0]) + krate(lines[1])
    return(rsum)

def change_speed(lines):
    try:
        if abs(kratesum(lines)) <= 0.5:
            v1 = 40
            v2 = 40       
        if kratesum(lines) < -0.5:
            v1 = 60
            v2 = -30
        if kratesum(lines) > 0.5:
            v1 = -30 
            v2 = 60
    except:
        v1 = v2 = 40
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
    fid = 0
    car.set_speed(30, 30)
    while fid < 50:
        image = np.empty((640 * 480 * 3,), dtype=np.uint8)
        input_file = "./img/"+str(fid)+".jpg"
        camera.capture(input_file, "rgb")
        input_image = mpimg.imread(input_file)
        output_file = "./img_out/"+str(fid)+".jpg"
        output_image = annotate_image_array(input_image)
        plt.imsave(output_file, output_image)
        time.sleep(0.5)
        print(input_file, output_file)
        fid = fid + 1
    car.set_speed(0, 0)