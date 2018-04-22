#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class HSVColorSegmentation(object):
    def __init__(self):

        rospy.init_node('hsv_color_segmentation')

        self.bridge = CvBridge()
        
        self.image_hsv_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.segment_image)
        self.image_sub = rospy.Subscriber("/hsv_values", Int32MultiArray, self.values_extract)
        self.image_hsv = rospy.Publisher("cam/hsv_seg_image", Image)

        self.image = None
        self.count = 0
        print "Started HSV Color Segmentation Node"

    def segment_image(self, data):

        try:
            # print "callback"
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
    def values_extract(self, data):

        vals = np.array(data.data)

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        print (vals, self.count)
        self.count = self.count + 1
        result = cv2.inRange(hsv, np.array([vals[0], vals[1], vals[2]]), np.array([vals[3], vals[4], vals[5]]))

        try:

            self.image_hsv.publish(self.bridge.cv2_to_imgmsg(result, "passthrough"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':

    obj = HSVColorSegmentation()
    rospy.spin()