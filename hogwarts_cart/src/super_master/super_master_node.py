#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, Int32MultiArray
from color_segmentation import ColorSegmentation
from pid_controller import PidController

from enum import Enum

class State(Enum):
    Waiting = 1
    Following = 2
    stop = 3

class SuperMasterNode(object):

    def __init__(self):

        rospy.init_node('super_master_node')

        self.color_segmentation = ColorSegmentation()
        self.pid_controller = PidController()

        self.current_state = State.Waiting
        self.rushing_speed = 50
        self.rushing_area = 8000

    def start_main_controller(self):

        rospy.sleep(5)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.current_state == State.Waiting:
                # do some waiting logic here.
                self.current_state = State.Following
            elif self.current_state == State.Following:
                # print self.color_segmentation.get_target_area()
                self.follow()
            rate.sleep()

    def follow(self):
        center_x = self.color_segmentation.get_center_target()
        area = self.color_segmentation.get_target_area()
        if area < 200 or area > (50000): #upper area is not super accurate
            # no object or too close
            self.pid_controller.idle()
            print "no object"
        elif area < self.rushing_area:
            # rushing the train to catch up speed
            print "rushing", self.rushing_speed, center_x, area
            self.pid_controller.set_speed(self.rushing_speed)
            self.pid_controller.start_pid(center_x)
        else:
            # train speed
            k = self.rushing_speed * self.rushing_area
            speed = k / area
            self.pid_controller.set_speed(speed)
            self.pid_controller.start_pid(center_x)
            print "following", speed, center_x, area
       
if __name__ == '__main__':
    mastar_node = SuperMasterNode()
    mastar_node.start_main_controller()
