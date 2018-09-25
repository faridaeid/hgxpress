#!/usr/bin/env python


import rospy
from std_msgs.msg import Int32, Int32MultiArray

from pid import PID


class PidController:
    def __init__(self, setpoint):

        self.speed_pub = rospy.Publisher("motor/speed_motors", Int32MultiArray)

        self.pid = PID(0.3, 0.0, 0.0)
        self.pid.set_setpoint(setpoint)
        self.constant_speed = 54


    def start_pid(self, center_x):

        pid_output = self.pid.update(center_x)

        left_motor_speed = self.constant_speed - pid_output
        right_motor_speed = self.constant_speed + pid_output

        speeds = Int32MultiArray()
        speeds.data = [right_motor_speed, left_motor_speed]

        self.speed_pub.publish(speeds)

    def update_setpoint(self, value):
        self.pid.set_setpoint(value)

    def set_speed(self, speed):
        self.constant_speed = speed


    def stop_pid(self):

        speeds = Int32MultiArray()
        break_speed = -40
        break_dur = 0.02

        speeds.data = [break_speed, break_speed]
        self.speed_pub.publish(speeds)
        rospy.sleep(break_dur)
        speeds.data = [0, 0]
        self.speed_pub.publish(speeds)

    def idle(self):
        speeds = Int32MultiArray()
        speeds.data = [0, 0]
        self.speed_pub.publish(speeds)



if __name__ == '__main__':
    obj = PidController()
