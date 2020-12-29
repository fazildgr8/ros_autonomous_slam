#!/usr/bin/python
'''
  Dynamic Path Planning Node
  ------------------------
  Created by Mohamed Fazil on 12/7/20.

  email: mohamedfazilsulaiman@gmail.com
  github: https://github.com/fazildgr8
'''
import roslib
roslib.load_manifest('gazebo_sim')
import rospy
import tf
import math
from math import cos,sin
import random
import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import sys
from pprint import pprint
import cv2

global robot_location, robot_rotation, final_goal_location, robot_start_location, final_path, global_map, goal_reached,global_data

laser_ranges = np.zeros((360,))
laser_intensities = np.zeros((360,))
final_path = None
global_map = np.zeros((384,384))
global_data = np.zeros((384*384,))
robot_start_location = [-2,-0.5,0]
robot_location = robot_start_location
robot_rotation = [0,0,0]
goal_reached = False
iter_break = False
final_goal_location = [10,10,0]
# final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]

##################### Perception Part #####################

def callback_laser(msg):
    '''
    Laser Msgs
    '''
    global laser_ranges, laser_intensities
    laser_ranges = np.array(msg.ranges)
    laser_intensities = np.array(msg.intensities)

def callback_odom(msg):
    '''
    Obtains Odometer readings and assigns to global Variables
    '''
    global robot_location, robot_rotation, robot_orientation
    location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    robot_location = location
    orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    rot = [roll, pitch, yaw]
    robot_rotation = rot
    robot_orientation = orientation
    # map_g = np.copy(global_map)
    # OccupancyGrid_publish(map_g[::-1]) # Publish the Occupancy Grid
    # goal_location_marker(final_goal_location) # Publish Goal 

def callback_map(msg):
    global global_data
    global_data = np.array(msg.data)

def callback_map_metadata(msg):
    global global_map,global_data
    height = msg.height
    width = msg.width
    global_map = global_data.reshape(width,height)

#################### Planning Part #########################


################### Motion Controll ########################


if __name__ == '__main__':
    rospy.init_node('Controll')
    rate = rospy.Rate(10.0)
    # Subscribe to /odom and laser sensor
    sub_laser = rospy.Subscriber('/scan', LaserScan, callback_laser) # Receive Laser Msg from /stage
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom) # Receive Odom readings
    sub_map = rospy.Subscriber('/map',OccupancyGrid,callback_map)
    sub_map_matadata = rospy.Subscriber('/map_metadata',MapMetaData,callback_map_metadata)
    # rospy.spin()
    while not rospy.is_shutdown():
        print(robot_location)
        # print(global_map)
        # # Display the resulting frame
        # cv2.imshow('frame',abs(global_map)/255)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break