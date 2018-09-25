#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#gryf 168-185 84-255 0-255
#raven 104-119 182-255 0-255
#slyth 85-99 57-255 0-255
#huff 16-25 40-255 0-255


#class FollowingCar(Enum):
#    Gryffindor = 0,
#    Slytherin = 1,
#    Hufflepuff = 2,
#    Ravenclaw = 3

# hsv_colors = [[np.array([0, 200, 100]), np.array([16, 255, 255])],
#               [np.array([85, 57, 0]), np.array([99, 255, 255])],
#               [np.array([16, 40, 0]), np.array([25, 255, 255])],
#               [np.array([103, 182, 0]), np.array([119, 255, 255])],
#               [np.array([85, 57, 60]), np.array([99, 255, 255])]]
#

# red 0-18 30-255 0-255
# green 77-100 39-255 0-255
# blue 100-115 113-255 90-255
# yellow 21-34 47-255 34-255

# red saved np.array([0, 30, 0]), np.array([18, 168, 255])

mask_day = [
    [np.array([0, 118, 0]), np.array([23, 230, 255])],
    [np.array([77, 39, 0]), np.array([100, 255, 255])],
    [np.array([21, 57, 34]), np.array([34, 255, 255])],
    [np.array([100, 113, 90]), np.array([115, 255, 255])],
    [np.array([81, 153, 146]), np.array([149, 248, 255])]
]

mask_night = [
    [np.array([129, 66, 41]), np.array([179, 130, 171])],
    [np.array([130, 50, 43]), np.array([179, 255, 255])],
    [np.array([96, 106, 194]), np.array([179, 255, 255])],
    [np.array([93, 169, 131]), np.array([179, 255, 255])],
    [np.array([81, 153, 146]), np.array([149, 248, 255])]
]

mask_dusk = [
    [np.array([0, 118, 0]), np.array([23, 230, 255])],
    [np.array([80, 46, 92]), np.array([99, 151, 255])],
    [np.array([83, 53, 203]), np.array([115, 255, 255])],
    [np.array([93, 169, 131]), np.array([179, 255, 255])],
    [np.array([68, 94, 148]), np.array([125, 211, 251])]
]

mask_test = [
    [np.array([20, 0, 205]), np.array([73, 143, 255])], # done
    [np.array([77, 39, 0]), np.array([100, 255, 255])], # done
    [np.array([119, 45, 34]), np.array([135, 255, 255])], # done
    [np.array([30, 50, 115]), np.array([133, 172, 255])], # done
    [np.array([81, 153, 146]), np.array([149, 248, 255])]
]



class ColorSegmentation:

    def __init__(self):
        self.bridge = CvBridge()
        self.following_color = 4
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.segment_image)
        self.cart_name = rospy.get_param('~cart_name')
        self.image_bound_seg = rospy.Publisher("cam/image_seg", Image)
        self.image_center_x = 0
        self.target_area = 0
        self.seg_image_cropped = None
        self.threshold = 20
        self.following_color = None

    def set_following_color(self, value):
        self.following_color = value

    def _color_segmentation(self, cv_image):

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        if self.following_color is None:
            seg_image = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([0, 0, 1]))
        else:

            # lower = np.array(mask_day[0][self.following_color][0])
            # upper = np.array(mask_day[0][self.following_color][1])
            # seg_image_mask1 = cv2.inRange(hsv, lower, upper)
            #
            # lower = np.array(mask_day[1][self.following_color][0])
            # upper = np.array(mask_day[1][self.following_color][1])
            # seg_image_mask2 = cv2.inRange(hsv, lower, upper)
            #
            # seg_image = cv2.bitwise_or(seg_image_mask1, seg_image_mask2)
            # print "self following = {}".format(self.following_color)

            # lower = np.array(mask_post_midnight[0][self.following_color][0])
            # upper = np.array(mask_post_midnight[0][self.following_color][1])
            seg_image = cv2.inRange(hsv, mask_test[self.following_color][0], mask_test[self.following_color][1])

            # lower = np.array(mask_day[1][self.following_color][0])
            # upper = np.array(mask_day[1][self.following_color][1])
            # seg_image_mask2 = cv2.inRange(hsv, lower, upper)
            #
            # seg_image = cv2.bitwise_or(seg_image_mask1, seg_image_mas


        return seg_image

    def _get_boundaries(self, seg_image, cv_image):
        # seg_image = cv2.dilate(seg_image, )
        output = cv2.connectedComponentsWithStats(seg_image, 4, cv2.CV_8U)
        stats = output[2]

        indx = 0
        if stats.shape[0] > 1:
            indx = np.argmax(stats[1:, 4]) + 1  # TODO here

        x, y, w, h, area = stats[indx]
        center_x = x + w // 2
        center_y = y + h // 2

        self.seg_image_cropped = seg_image[y:y + h, x:x + w]
        self.cropped_image = cv_image[y:y + h, x:x + w]
        self.image_center_x = center_x
        self.image_center_y = center_y
        self.target_area = w * h


    def segment_image(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        seg_image = self._color_segmentation(cv_image)
        self._get_boundaries(seg_image, cv_image)

    def get_center_target(self):
        return self.image_center_x

    def get_target_area(self):
        return self.target_area


if __name__ == '__main__':
    obj = ColorSegmentation()
    rospy.spin()
