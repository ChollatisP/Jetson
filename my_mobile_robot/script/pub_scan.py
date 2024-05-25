#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_cb(data):
    
    data.ranges = data.ranges[1:]
    # Publish the modified laser scan message
    pub_scan.publish(data)
if __name__ == '__main__':
    rospy.init_node('laser_scan_modifier')

    # Subscriber for the original laser scan topic
    rospy.Subscriber('/scan', LaserScan, scan_cb)

    # Publisher for the modified laser scan topic
    pub_scan = rospy.Publisher('/new_scan', LaserScan, queue_size=10)

    rospy.spin()
