#!/usr/bin/env python


import rospy
from initial_pose_publisher import InitialPosePublisher
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32MultiArray, Int32
import actionlib
import detach
import numpy as np
from stations import station_poses, StationDirection


class NavigationController(object):

    def __init__(self):

        rospy.init_node('navigation_controller_node')
        initial = InitialPosePublisher()
        self.detach_controller = detach.Detach()
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.next_goal_publisher = rospy.Publisher('navigation/next_goal', Int32)
        self.park_publisher = rospy.Publisher('navigation/park_request', Int32)
        self.path_subscriber = rospy.Subscriber('navigation/path', Int32MultiArray, self.set_path)
        self.client.wait_for_server()
        self.path = None
        self.cart_number = 1 # get from upper layer
        self.current_pose_index = 0
        self.goal_threshold_x = 0.1
        self.goal_threshold_y = 0.1
        self.next_goal_pose = None
        self.next_goal = None

        rospy.sleep(3)
        # self.detach_flag = False

    def set_path(self, data):
        self.current_pose_index = 0
        self.path = np.array(data.data)
        self.set_goal()


    def active_cb(self):
        rospy.loginfo("active db")

    def feedback_cb(self, feedback):

        x = feedback.base_position.pose.position.x
        y = feedback.base_position.pose.position.y

        self.detach_controller.current_pose_index = feedback.base_position

        distx = abs(self.next_goal_pose.position.x - x)
        disty = abs(self.next_goal_pose.position.y - y)

        if distx < self.goal_threshold_x and disty < self.goal_threshold_y and self.current_pose_index < len(self.path):

            # self.detach_controller.goal = feedback.base_position
            self.set_goal()

        # self.detach_controller.try_detach()

    def done_cb(self, status, result):
        # self.goalsCount += 1
        # print(self.goalsCount)

        if self.path is not None and self.current_pose_index < len(self.path):
            rospy.loginfo("Status " + str(status))
            self.set_goal()
        else:
            self.park_publisher.publish(1)


    def set_goal(self):
        self.current_pose_index += 1
        if self.current_pose_index < len(self.path):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            self.next_goal = StationDirection(self.path[self.current_pose_index])
            self.next_goal_pose = station_poses[self.next_goal]

            goal.target_pose.pose = self.next_goal_pose

            print "Next goal = ", self.next_goal

            self.next_goal_publisher.publish(self.next_goal.value)

            self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)


if __name__ == '__main__':
    motor_node = NavigationController()
    rospy.spin()
