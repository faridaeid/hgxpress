#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32, Int32MultiArray
from navigation_path_finder import PathFinder
from enum import Enum
import numpy as np
from stations import Station, request_mapping, destination_mapping, StationDirection


class Cart(Enum):
    Gryffindor = 0
    Slytherin = 1
    Hufflepuff = 2
    Ravenclaw = 3



class RequestHandler(object):
    def __init__(self):

        rospy.init_node('request_handler_node')

        self.train = [Cart.Gryffindor]
        self.last_cart_index = 0
        self.train_max = 4
        self.next_goal = StationDirection.PlatformFor

        self.requests = []
        self.destinations = []
        self.directions_info = {}

        self.path_finder = PathFinder()

        self.next_goal_navigation = rospy.Subscriber('navigation/next_goal', Int32, self.set_next_goal)
        self.next_goal_navigation = rospy.Subscriber('navigation/park_request', Int32, self.handle_park_request)
        self.follow_color_slyth = rospy.Publisher("slyth/follow_color", Int32)
        self.follow_color_huff = rospy.Publisher("huff/follow_color", Int32)
        self.follow_color_raven = rospy.Publisher("raven/follow_color", Int32)
        self.request_subscriber = rospy.Subscriber('requests', Int32MultiArray, self.handling_requests)
        self.detach_cart_publisher = rospy.Publisher('navigation/detach_cart', Int32MultiArray)

    def set_next_goal(self, data):
        self.next_goal = StationDirection(data.data)

        print "Setting next goal of request to {}".format(self.next_goal)
        if self.next_goal in self.directions_info:

            print "Next goal is a pick up = {}".format(self.next_goal)

            info = self.directions_info[self.next_goal]
            last_cart = self.train[-1]
            self.train.append(info)
            self.publish_to_cart(info, last_cart)

    def handle_park_request(self):
        self.path_finder.publish_path(self.next_goal, [StationDirection.PlatformBack])


    def publish_to_cart(self, cart, last_cart):
        if cart == Cart.Slytherin:
            self.follow_color_slyth.publish(last_cart.value)
        elif cart == Cart.Hufflepuff:
            self.follow_color_huff.publish(last_cart.value)
        elif cart == Cart.Ravenclaw:
            self.follow_color_raven.publish(last_cart.value)

    def remove(self, cart_id):
        cart = Cart(cart_id)
        cart_index = self.train.index(cart)
        self.train.pop(cart_index)

        # we should publish somewhere to let the previous car follow the following one and stop the jetson

    def handling_requests(self, data):
        print "hereee"

        request = np.array(data.data)

        for i in range(0, len(request), 2):
            cart = Cart(request[i])
            station = request_mapping[Station(request[i+1])]
            self.directions_info[station] = cart
            self.requests.append(station)

        self.path_finder.publish_path(self.next_goal, self.requests)


        # cart = Cart(request[0])
        # pick_up_station = request_mapping[Station(request[1])]
        # drop_off_station = destination_mapping[Station(request[2])]
        #
        # print "Request From", cart.value
        # if len(self.train) == self.train_max:
        #     print "Max number of carts attached"
        # else:
        #     self.directions_info[pick_up_station] = ('pick_up', cart)
        #     self.directions_info[drop_off_station] = ('drop_off', cart)
        #     self.requests.append(pick_up_station)
        #     self.destinations.append(drop_off_station)
        #     self.path_finder.publish_path(self.next_goal, self.requests, self.destinations)

            # last_cart = self.train[-1]
            # self.train.append(cart)
            # self.publish_tocart(cart, last_cart)

        # if self.last_cart_index < len(self.train) - 1:
        #     self.last_cart_index += 1
        #     self.train[self.last_cart_index] = caller_cart


if __name__ == '__main__':
    request_handler = RequestHandler()
    rospy.spin()
