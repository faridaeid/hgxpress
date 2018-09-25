#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Bool, Int32
from color_segmentation import ColorSegmentation
from pid_controller import PidController

from enum import Enum


class State(Enum):
    Follow = 1
    Detach = 2
    Idle = 3
    Stop = 4
    Nothing = 5


class MasterNode(object):

    def __init__(self):

        self.prev_area = 0
        rospy.init_node('master_node')
        self.cart_name = rospy.get_param('~cart_name')
        self.pub_motor = rospy.Publisher("motor/speed_motors", Int32MultiArray)
        self.detach_order = rospy.Subscriber('/' + self.cart_name + '/detach', Bool, self.detach)
        self.follow_order = rospy.Subscriber('/' + self.cart_name + '/follow_color', Int32, self.set_following_color)
        self.color_segmentation = ColorSegmentation()
        self.pid_controller = PidController(160)
        self.motor_speed = Int32MultiArray()
        self.current_state = State.Idle
        self.stop = False
        self.rushing_area = 8000
        self.max_area = 40000
        self.min_area = 500
        self.max_speed = 80
        self.rotating_speed = 40
        self.speed_offset = (self.min_area * self.max_speed) / self.max_area
        self.area_count = 0
        self.start_main_controller()

    def detach(self, data):
        if data.data:
            self.color_segmentation.set_following_color(4)
            self.current_state = State.Detach

    def set_following_color(self, data):
        self.color_segmentation.set_following_color(data.data)
        self.current_state = State.Follow

    def rotate(self, rotate_right):

        print "rotating"

        if rotate_right:
            # rotate right
            self.motor_speed.data = ([self.rotating_speed * -1,  self.rotating_speed * 1])
        else:
            # rotate left
            self.motor_speed.data = ([self.rotating_speed * 1,  self.rotating_speed * -1])

        self.pub_motor.publish(self.motor_speed)

    def park(self):
        min_sign_area = 400
        max_sign_area = 5000
        no_sign_area = 50000
        pid_setpoint_offset = 80
        rotate_right = False
        # pid_setpoint = self.image_center + pid_setpoint_offset
        sign_area = self.color_segmentation.get_target_area()

        print "sign area = {}".format(sign_area)

        if sign_area < min_sign_area or sign_area > no_sign_area:
            self.rotate(rotate_right)
        elif sign_area < max_sign_area:
            self.pid_controller.update_setpoint(240)
            self.follow(min_sign_area, max_sign_area)
        else:
            self.current_state = State.Stop

    def start_main_controller(self):

        rospy.sleep(5)
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.current_state == State.Follow:
                print "following"
                self.pid_controller.update_setpoint(160)
                self.follow(self.min_area, self.max_area)
            elif self.current_state == State.Detach:
                print "detaching"
                self.park()
            elif self.current_state == State.Idle:
                print "idle"
            elif self.current_state == State.Stop:
                self.stop()

            rate.sleep()

    def follow(self, min_area, max_area):

        center_x = self.color_segmentation.get_center_target()
        area = self.color_segmentation.get_target_area()

        print "area = {}".format(area)

        # if abs(self.prev_area - area) > 100:
        #     pass

        self.prev_area = area

        if area < min_area or area > max_area:  # upper area is not accurate
            if not self.stop:
                self.pid_controller.idle()
                self.stop = True
            # self.current_state = State.Idle
            # print "following"
        else:

            speed = int(((max_area - area) * self.max_speed)/self.max_area)
            self.stop = False
            # speed = int(((max_area - area) * self.max_speed)/self.max_area)
            print "speed = {}".format(speed)
            # print "speed = {}".format(speed)
            self.pid_controller.set_speed(speed)
            self.pid_controller.start_pid(center_x)

    def stop(self):
        min_area = 600
        no_sign_area = 50000
        curr_area = self.color_segmentation.get_target_area()

        if curr_area > min_area:
            self.motor_speed.data = ([30, 30])
            self.pub_motor.publish(self.motor_speed)
        elif curr_area > no_sign_area:
            self.current_state = State.Idle
        else:
            self.current_state = State.Idle


if __name__ == '__main__':
    master_node = MasterNode()
    master_node.start_main_controller()
