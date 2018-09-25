#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String, Int32
from enum import Enum
import numpy as np


class FollowingCar(Enum):
    Gryffindor = 0,
    Slytherin = 1,
    Hufflepuff = 2,
    Ravenclaw = 3


class RequestCart(object):
    def __init__(self):

        self.request_cart_pub = rospy.Publisher("requests", Int32)




if __name__ == '__main__':
    request_car = RequestCart()
    rospy.spin()