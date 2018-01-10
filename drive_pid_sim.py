#! /usr/bin/python
# coding=utf-8

import argparse
import base64
from datetime import datetime
import os
import shutil

import numpy as np
import socketio
import eventlet
import eventlet.wsgi
from PIL import Image
from flask import Flask
from io import BytesIO

import pidcontroller
from lane_lines import *

krate_sum=[ 0 for i in range(10) ]

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

def krate(line):
    # compute the sign of the slop of the line
    rate=0
    try:
        rate = (line[0] - line[2]) / (line[1] - line[3])
    except:
        rate = 0
    return round(rate, 4)

def kratesum(lines):
    global krate_sum
    rsum = krate(lines[0]) + krate(lines[1])
    krate_sum.pop(0)
    krate_sum.append(rsum)
    result=np.mean(krate_sum)
    return(result)

last_left_rate=0
last_right_rate=0

def kratesum1(lines):
    global last_left_rate,last_right_rate
    sum=0
    if lines is not None:
        if lines[0] is not None:
            last_left_rate = krate(lines[0])
            if lines[1] is None:
                if last_left_rate<1:
                    last_right_rate=last_left_rate
        if lines[1] is not None:
            last_right_rate = krate(lines[1])
            if lines[0] is None:
                if last_right_rate>-1:
                    last_left_rate=last_right_rate
    sum=last_left_rate+last_right_rate
    return sum

sio = socketio.Server()
app = Flask(__name__)
model = None
prev_image_array = None


class SimplePIController:
    def __init__(self, Kp, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.set_point = 0.
        self.error = 0.
        self.integral = 0.

    def set_desired(self, desired):
        self.set_point = desired

    def update(self, measurement):
        # proportional error
        self.error = self.set_point - measurement

        # integral error
        self.integral += self.error

        return self.Kp * self.error + self.Ki * self.integral


controller = SimplePIController(0.1, 0.002)
set_speed = 9
controller.set_desired(set_speed)
target_direction=0
angle_controller = SimplePIController(0.1, 0.002)
angle_controller.set_desired(target_direction)
#angle_controller = pidcontroller.PID(0.1, 0.002, 0.001)

@sio.on('telemetry')
def telemetry(sid, data):
    if data:
        # The current steering angle of the car
        steering_angle = data["steering_angle"]
        # The current throttle of the car
        throttle = data["throttle"]
        # The current speed of the car
        speed = data["speed"]
        # The current image from the center camera of the car
        imgString = data["image"]
        image = Image.open(BytesIO(base64.b64decode(imgString)))
        image_array = np.asarray(image)
        #steering_angle = float(model.predict(image_array[None, :, :, :], batch_size=1))
        lines=stage_detect(image_array)
        current_direction = kratesum1(lines)
        steering_angle = angle_controller.update(current_direction)
        #error = target_direction - current_direction
        #steering_angle = angle_controller.Update(error)

        #print('Setting the angle to %f' % correction)
        #steering_angle=0.1
        throttle = controller.update(float(speed))
        print(steering_angle)
        send_control(steering_angle, throttle)

        # save frame
        if args.image_folder != '':
            timestamp = datetime.utcnow().strftime('%Y_%m_%d_%H_%M_%S_%f')[:-3]
            image_filename = os.path.join(args.image_folder, timestamp)
            image.save('{}.jpg'.format(image_filename))
    else:
        # NOTE: DON'T EDIT THIS.
        sio.emit('manual', data={}, skip_sid=True)


@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)
    send_control(0, 0)


def send_control(steering_angle, throttle):
    sio.emit(
        "steer",
        data={
            'steering_angle': steering_angle.__str__(),
            'throttle': throttle.__str__()
        },
        skip_sid=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Remote Driving')
    # parser.add_argument(
    #     'model',
    #     type=str,
    #     help='Path to model h5 file. Model should be on the same path.'
    # )
    parser.add_argument(
        'image_folder',
        type=str,
        nargs='?',
        default='',
        help='Path to image folder. This is where the images from the run will be saved.'
    )
    args = parser.parse_args()

    # check that model Keras version is same as local Keras version
    # f = h5py.File(args.model, mode='r')
    # model_version = f.attrs.get('keras_version')
    # keras_version = str(keras_version).encode('utf8')
    #
    # if model_version != keras_version:
    #     print('You are using Keras version ', keras_version,
    #           ', but the model was built using ', model_version)
    #
    # model = load_model(args.model)

    if args.image_folder != '':
        print("Creating image folder at {}".format(args.image_folder))
        if not os.path.exists(args.image_folder):
            os.makedirs(args.image_folder)
        else:
            shutil.rmtree(args.image_folder)
            os.makedirs(args.image_folder)
        print("RECORDING THIS RUN ...")
    else:
        print("NOT RECORDING THIS RUN ...")

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)