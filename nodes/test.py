#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction
import numpy as np
import actionlib

if __name__ == '__main__':
    rospy.init_node('Tester')
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        client.wait_for_server()
        print(client.get_state())

        
