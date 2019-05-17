#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from ctrl_pkg.msg import ServoCtrlMsg
from inference_pkg.msg import InferResultsArray, InferResults
from sensor_msgs.msg import Image
import cv2
import json
import rospy
import numpy as np

import signal
import sys

# global parameter for proportional controller
k_angle = 1

class Proportional_controller:
    def __init__(self):
        self.turn_possibility = 0.0
        self.angle = 0.0
        self.throttle = 0.0

        self.video_listener()
        self.pub = rospy.Publisher("manual_drive", ServoCtrlMsg, queue_size = 10)

    def video_cb(self, data):
	    import time
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
             print(e)
        image = cv2.resize(cv_image, (160, 120))
        self.turn_possibility = self.steering(image)
        # proportional controller to generate steering angle and throttle
        self.angle, self.throttle = self.controller(self.turn_possibility) 
	    self.inputs_publisher()

    def controller(self, turn_possibility):
        return -k_angle*turn_possibility, 0.7

    def steering(self, img):
        # takes image and returns a steering "angle" where stearing angle is between [-1,1]
        # negative numbers means go left
        # positive numbers means go right
        lower_green = np.array([40,10,60])
        upper_green = np.array([100,255,255])
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # look for white pixel in first and last 20 columms 
        row_left, _ = np.where(mask[:,:10]==255)
        row_right, _ = np.where(mask[:,-10:]==255)

        # Robust choice for pixels
        if len(row_left) < 5:
            left_most = 0
            right_most = np.median(row_right)
        elif len(row_right) < 5:
            right_most = 0
            left_most = np.median(row_left)
        else:
            left_most = np.median(row_left)
            right_most = np.median(row_right)

        turn = left_most - right_most
        turn /= 120 # normalize between [-1,1]
        
        return turn

    def video_listener(self):
        rospy.Subscriber("video_mjpeg", Image, self.video_cb)
	

    def inputs_publisher(self):
        state = ServoCtrlMsg()
        state.angle = self.angle
        state.throttle = self.throttle
	    print("state",state)
	    self.pub.publish(state)

if __name__ == '__main__':
	try:
        rospy.init_node('proportional_controller', anonymous=True)
	    nav_node = Proportional_controller()
        rospy.spin()
	except rospy.ROSInterruptException:
		pass