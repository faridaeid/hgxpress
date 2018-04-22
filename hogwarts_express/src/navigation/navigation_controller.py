#!/usr/bin/env python


import rospy
from initial_pose_publisher import InitialPosePublisher
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, Pose, Quaternion


class NavigationController(object):

    def __init__(self):
        rospy.init_node('navigation_controller_node')
        initial_pose_publisher = InitialPosePublisher()

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.goals = dict()
        self.goals['1'] = Pose(Point(103.551, 101.058, 0.000), Quaternion(.000, 0.000, 0.022, 1.000))
        self.goals['2'] = Pose(Point(106.319, 100.416, 0.000), Quaternion(0.000, 0.000, 0.023, 1.000))
        self.goals['3'] = Pose(Point(108.712, 102.883, 0.000), Quaternion(0.000, 0.000, 0.728, 0.686))
        self.goals['4'] = Pose(Point(111.085, 104.490, 0.000), Quaternion(0.000, 0.000, 0.020, 1.000))

        self.goalsCount = 1
        self.set_goal()

    def active_cb(self):
        rospy.loginfo("active db")

    def feedback_cb(self, feedback):
        rospy.loginfo("feedback db"+ str(feedback))

    def done_cb(self, status, result):
        self.goalsCount += 1

        if self.goalsCount  < len(self.goals):
            rospy.loginfo("Status " + str(status))
            self.set_goal()

    def set_goal (self):

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.goals[str(self.goalsCount)]
        self.client.send_goal(goal,self.done_cb, self.active_cb, self.feedback_cb)
        rospy.spin()


if __name__ == '__main__':
    motor_node = NavigationController()


