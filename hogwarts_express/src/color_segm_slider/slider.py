#!/usr/bin/env python


import rospy
import cv2
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge


class Slider(object):

    def __init__(self):

        rospy.init_node('slider')

        self.bridge = CvBridge()
        self.hsv_values = rospy.Publisher("/hsv_values", Int32MultiArray, queue_size=2)

        cv2.namedWindow('result')
        
        # Creating track bar
        cv2.createTrackbar('lower_h', 'result', 0, 179, nothing)
        cv2.createTrackbar('lower_s', 'result', 0, 255, nothing)
        cv2.createTrackbar('lower_v', 'result', 0, 255, nothing)

        cv2.createTrackbar('upper_h', 'result', 0, 179, nothing)
        cv2.createTrackbar('upper_s', 'result', 0, 255, nothing)
        cv2.createTrackbar('upper_v', 'result', 0, 255, nothing)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            # get info from track bar and apply to result
            lo_h = cv2.getTrackbarPos('lower_h', 'result')
            lo_s = cv2.getTrackbarPos('lower_s', 'result')
            lo_v = cv2.getTrackbarPos('lower_v', 'result')

            up_h = cv2.getTrackbarPos('upper_h', 'result')
            up_s = cv2.getTrackbarPos('upper_s', 'result')
            up_v = cv2.getTrackbarPos('upper_v', 'result')

            values = Int32MultiArray()

            values.data = [lo_h, lo_s, lo_v, up_h, up_s, up_v]

            self.hsv_values.publish(values)

            k = cv2.waitKey(1) & 0xFF
            rate.sleep()
        cv2.destroyAllWindows()


def nothing(x):
    pass


if __name__ == '__main__':
    obj = Slider()
    rospy.spin()
