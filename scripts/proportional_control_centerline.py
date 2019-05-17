#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from ctrl_pkg.msg import ServoCtrlMsg
from inference_pkg.msg import InferResultsArray, InferResults
from sensor_msgs.msg import Image
import cv2
import json
import rospy
import numpy as np
from imutils import perspective
import os
import signal
import sys
import imutils
from time import time

# global parameter for proportional controller
k_angle = 0.5
direction = -1

calibration_velocity = 0
calibration_angle = 0.3

record = True
experiment_name = 'proportional_controller'

class Proportional_controller:
    def __init__(self):
        self.turn_possibility = 0.0
        self.angle_prev = 0.0
        self.angle_next = 0.0
        self.throttle = 0.7
        self.count = 0
	self.time = str(time())
        self.video_listener()
        self.pub = rospy.Publisher("manual_drive", ServoCtrlMsg, queue_size = 10)

    def video_cb(self, data):
        self.count += 1
        self.time = str(time())
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
             print(e)
        image = cv2.resize(cv_image, (160, 120))
        self.turn_possibility = self.steering(image)
        self.angle_next, self.throttle = self.controller(self.turn_possibility)
        self.angle_prev = self.angle_next
        self.inputs_publisher()

        if record:
            fname = 'data/'+ experiment_name + '/images/'  + self.time + '.png'
            cv2.imwrite(fname, image)

    def controller(self, turn_possibility):
        angle_next = direction * turn_possibility
        return angle_next, self.throttle

    def steering(self, img):

        if self.count <= 50:
            self.throttle = 0.6
        if self.count <= 100:
            self.throttle = 0.5
        if self.count <= 200:
            self.throttle = 0

        # takes image and returns a steering "angle" where stearing angle is between [-1,1]
        # negative numbers means go right
        # positive numbers
        lower_blue = np.array([100,100,100])
        upper_blue = np.array([110,255,255])

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        half_x = 80
        full_y = 120

        h, w = np.shape(mask)

        major = cv2.__version__.split('.')[0]
        if major == '3':
            _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        else:
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 1:
            self.throttle = 0.7
            c = contours[1]
        else:
            self.throttle = 0
            return 0

        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")

        # order the points in the contour such that they appear
        # in top-left, top-right, bottom-right, and bottom-left
        # order, then draw the outline of the rotated bounding
        # box
        box = perspective.order_points(box)

        # calculate moments for each contour

        M = cv2.moments(c)

        # calculate x,y coordinate of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        else:
            cX = (box[1][0] + box[0][0])//2
            cY = (box[1][1] + box[0][1])//2

        y_new_coord = full_y - cY
        x_new_coord = cX - half_x

        angle = np.arctan(x_new_coord/y_new_coord) / np.pi

        if angle <-0.2:
            angle = -0.8
        elif angle > 0.2:
            angle = 1.0
        else:
            angle = 0.0

        return angle

    def video_listener(self):
        rospy.Subscriber("video_mjpeg", Image, self.video_cb)


    def inputs_publisher(self):
        state = ServoCtrlMsg()
        state.angle = self.angle_next
        state.throttle = direction * self.throttle - calibration_velocity
        print("state",state)
        self.pub.publish(state)

if __name__ == '__main__':
    if record:
        if not os.path.exists('data'):
            os.makedirs('data')
        if not os.path.exists('data/' + experiment_name):
            os.makedirs('data/' + experiment_name)
        if not os.path.exists('data/' + experiment_name + '/images'):
            os.makedirs('data/' + experiment_name + '/images')


    try:
        rospy.init_node('proportional_controller', anonymous=True)
        nav_node = Proportional_controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
