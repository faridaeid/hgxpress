#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


class LidarRepublisher(object):
    def __init__(self):
        rospy.init_node('lidar_republisher_node')
        self.sub_scan = rospy.Subscriber("unfiltered_scan", LaserScan, self.check_scan)
        self.pub_new_scans = rospy.Publisher('scan', LaserScan)
        self.last_valid_scan = None

    def check_scan(self, data):
        ranges = np.array(data.ranges, np.float32)
        ranges_length = ranges.size
        infinity_count = np.count_nonzero(ranges == np.inf)
        ranges[305:325] = np.inf
        if infinity_count > 0.7 * ranges_length:
            rospy.loginfo("{} out of {}".format(infinity_count, ranges.size))
        else:
            self.last_valid_scan = data
            self.last_valid_scan.ranges = ranges

        self.pub_new_scans.publish(self.last_valid_scan)


if __name__ == '__main__':
    lidar_node = LidarRepublisher()
    rospy.spin()
