#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ColorSegmentation:
    def __init__(self):

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.segment_image)
        # was slowing the pi some how
#        self.image_bound = rospy.Publisher("cam/image_raw", Image)
#        self.image_bound_canny = rospy.Publisher("cam/image_canny", Image)
#        self.image_bound_crooped = rospy.Publisher("cam/image_cropped", Image)
        self.image_bound_seg = rospy.Publisher("cam/image_seg", Image)

        self.image_center_x = 0
        self.target_area = 0
        self.seg_image_cropped = None

        self.threshold = 20

    def _color_segmentation(self, cv_image):

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #seg_image = cv2.inRange(hsv, np.array([50, 100, 50]), np.array([85, 255, 160])) # ???
        #seg_image = cv2.inRange(hsv, np.array([90, 90, 17]), np.array([110, 255, 255])) # blue i sink

        seg_image = cv2.inRange(hsv, np.array([27, 83, 97]), np.array([91, 255, 255]))
        # seg_image = cv2.medianBlur(seg_image, 11)

        return seg_image

    def _get_boundaries(self, seg_image, cv_image):

        output = cv2.connectedComponentsWithStats(seg_image, 4, cv2.CV_8U)
        stats = output[2]

        indx = 0
        if stats.shape[0] > 1:
            indx = np.argmax(stats[1:, 4]) + 1  #TODO here

        x, y, w, h, area = stats[indx]
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 0), thickness=2, lineType=8, shift=0)
        center_x = x + w // 2
        center_y = y + h // 2

        # cv2.circle(cv_image, (center_x, center_y), 2, (0,0,255), 2)
        self.seg_image_cropped = seg_image[y:y+h, x:x+w]
        self.cropped_image = cv_image[y:y+h, x:x+w]
        self.image_center_x = center_x
        self.image_center_y = center_y

        # print center_x

        cv2.circle(cv_image, (center_x, center_y), 2,  (0,0,255), 2)
        cv2.circle(cv_image, (center_x, center_y), 2,  (0,0,255), 2)

        self.target_area = w*h

        try:
#
#            self.image_bound.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#            self.image_bound_crooped.publish(self.bridge.cv2_to_imgmsg(self.cropped_image, "bgr8"))
            self.image_bound_seg.publish(self.bridge.cv2_to_imgmsg(seg_image, "passthrough"))
#
        except CvBridgeError as e:
            print(e)

    def segment_image(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        seg_image = self._color_segmentation(cv_image)

        self._get_boundaries(seg_image, cv_image)

    def get_center_target(self):
        return self.image_center_x

    def get_target_area(self):
        return self.target_area


if __name__ == '__main__':
    obj = ColorSegmentation()
    rospy.spin()
