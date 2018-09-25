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
        print vals

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        print (vals, self.count)
        self.count = self.count + 1
        print(vals)
        result = cv2.inRange(hsv, np.array([vals[0], vals[1], vals[2]]), np.array([vals[3], vals[4], vals[5]]))
        # res = cv2.GaussianBlur(result, (5,5)  , 0)


        mask1 = cv2.inRange(hsv, np.array([138, 67, 72]), np.array([179, 191, 160]))
        mask2 = cv2.inRange(hsv, np.array([143, 26, 192]), np.array([179, 169, 255]))
        mask3 = cv2.inRange(hsv, np.array([25, 0, 19]), np.array([42, 152, 255]))


        # result = cv2.bitwise_or(mask1, mask2)

        try:
            #
            # cv2.imshow('hsv', result)
            # cv2.waitKey(1)

            self.image_hsv.publish(self.bridge.cv2_to_imgmsg(result, "8UC1"))

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':

    obj = HSVColorSegmentation()
    rospy.spin()