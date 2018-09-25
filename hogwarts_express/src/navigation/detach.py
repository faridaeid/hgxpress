from enum import Enum
import rospy
from std_msgs.msg import Int32, Bool, Int32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion

import numpy as np

class Cart(Enum):
    Gryffindor = 0
    Slytherin = 1
    Hufflepuff = 2
    Ravenclaw = 3


class Detach(object):

    def __init__(self):

        self.detach_slyth = rospy.Publisher("slyth/detach", Bool)
        self.detach_huff = rospy.Publisher("huff/detach", Bool)
        self.detach_raven = rospy.Publisher("raven/detach", Bool)

        self.detach_subscriber = rospy.Subscriber('navigation/detach_cart', Int32MultiArray, self.set_detach_info)

        self.current_pose = None
        self.detach_info = None
        self.goal = None


    def publish_to_cart(self, cart):

        if cart == Cart.Slytherin:
            self.detach_slyth.publish(True)
        elif cart == Cart.Hufflepuff:
            self.detach_huff.publish(True)
        elif cart == Cart.Ravenclaw:
            self.detach_raven.publish(True)

    def set_detach_info(self, data):
        self.detach_info = np.array(data.data)


    def try_detach(self):

        if self.detach_info is not None:

            cart_number = self.detach_info[1]

            print "Detcaching"
            # print type(self.goal), type(self.current_pose)
            goal_x = self.goal.pose.position.x
            goal_y = self.goal.pose.position.y

            # print 'Goal x: ' , goal_x, ' Goal y: ', goal_y

            current_x = self.current_pose.pose.position.x
            current_y = self.current_pose.pose.position.y

            # print 'curr x: ' , current_x, ' curr y: ', current_y

            dist_x = abs(current_x - goal_x)
            dist_y = abs(current_y - goal_y)

            dist = np.sqrt(dist_x * dist_x + dist_y * dist_y)

            print "distance", dist

            min_dist = 1.45 * cart_number + ((cart_number - 1)*0.7)

            print "According to the condition it will detach"

            if dist > min_dist:
                self.goal = None
                self.publish_to_cart(Cart(self.detach_info[0]))
                self.detach_info = None
