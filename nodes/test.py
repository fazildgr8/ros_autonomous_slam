#!/usr/bin/python
import roslib
roslib.load_manifest('gazebo_sim')
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

global laser_intensities,laser_ranges
laser_ranges = np.zeros((360,))
laser_intensities = np.zeros((360,))

def callback_laser(msg):
    '''
    Laser Msgs
    '''
    global laser_ranges, laser_intensities
    laser_ranges = np.array(msg.ranges)
    laser_intensities = np.array(msg.intensities)

if __name__ == '__main__':
    global laser_ranges,laser_intensities
    rospy.init_node('Test')
    rate = rospy.Rate(10.0)

    sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
    while not rospy.is_shutdown():
        laser_ranges_cvt = np.copy(laser_ranges)
        for i in range(len(laser_ranges_cvt)):
            if(laser_ranges_cvt[i]==float('inf')):
                laser_ranges_cvt[i] = 0
        left = laser_ranges_cvt[0:179]
        right = laser_ranges_cvt[180:]
        front = laser_ranges_cvt[89:269]
        print(front.shape)

        
