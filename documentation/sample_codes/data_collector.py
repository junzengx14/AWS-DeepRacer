'''
source /opt/ros/kinetic/setup.bash
source /opt/aws/deepracer/setup.bash
source /opt/aws/intel/dldt/bin/setupvars.sh

#cat results.json | jq '.data[] .label'

'''

#!/usr/bin/env python
from cv_bridge import CvBridge, CvBridgeError
from ctrl_pkg.msg import ServoCtrlMsg
from inference_pkg.msg import InferResultsArray, InferResults
from sensor_msgs.msg import Image
import cv2
import json
import rospy


import signal
import sys
def signal_handler(sig, frame):
    print('You pressed Ctrl+C!... wait until we sync the json data to disk')
    print (nav_node.counter)
    with open('/home/deepracer/data/results.json', 'w') as outfile:
        json.dump(nav_node.data, outfile)
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class DataCollector:
    def __init__(self):
        self.ctrl_listener()
        self.infer_listener()
        self.counter = 0
        self.ctrl_state = {'throttle': 0.0, 'angle': 0.0}
        self.data = {}
        self.data['data'] = []
        self.label = -1
	self.results = []

    def ctrl_cb(self, data):
        self.ctrl_state['throttle'] = data.throttle
        self.ctrl_state['angle'] = data.angle

    def video_cb(self, data):
	import time
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(data.img, "bgr8")
        except CvBridgeError as e:
             print(e)
        image = cv2.resize(cv_image, (160, 120))
        image_name = 'image-%05d' % (self.counter)
        curr_state = {'image_name': image_name, 'throttle': self.ctrl_state['throttle'],
                      'angle': self.ctrl_state['angle'], 'time':time.time(),  
			        'label': self.label, 'results': self.results}

        self.data['data'].append(curr_state)

	# reduce latency, perhaps store in memory or async write
        cv2.imwrite('/home/deepracer/data/{}.jpg'.format(image_name), image)

        # Increment the counter
        self.counter += 1
    
    def inference_cb(self, data):
        label = -1
        prob = 0.0
        probs = []
        for result in data.results:
            probs.append(result.classProb)
            if result.classProb > prob:
                prob = result.classProb
                label = result.classLabel
       	self.label = label 
	self.results = probs 

        self.video_cb(data)

    def ctrl_listener(self):
        rospy.Subscriber("auto_drive", ServoCtrlMsg, self.ctrl_cb)

    #def video_listener(self):
    #    rospy.Subscriber("video_mjpeg", Image, self.video_cb)
    
    def infer_listener(self):
        rospy.Subscriber("rl_results", InferResultsArray, self.inference_cb)

if __name__ == '__main__':
        rospy.init_node('data_collect_node', anonymous=True)
        nav_node = DataCollector()
        rospy.spin()

