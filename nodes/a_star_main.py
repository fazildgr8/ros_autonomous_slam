#!/usr/bin/python
'''
  ROS A star Controll Node
  ------------------------
  Created by Mohamed Fazil on 12/7/20.
  email: mohamedfazilsulaiman@gmail.com
  github: https://github.com/fazildgr8
'''
import roslib
roslib.load_manifest('lab4')
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
################### Classes #####################################
class Node:
    '''
    Class which stores the node location,parent and costs
    '''
    def __init__(self,position):
        self.location = position
        self.prev = None
        self.g_cost = 0
        self.h_cost = 0
        self.f_cost = 0
        self.Occupied = False
    def update_cost(self,g,h):
        self.g_cost = g
        self.h_cost = h
        self.f_cost = g+h
    def __eq__(self, node):
        return self.location == node.location

################### Global Variables ####################################
global robot_location, robot_rotation, final_goal_location, robot_start_location, global_map, final_path, map_odom_trans, goal_reached

# Load the Map as an Array 0 - Empty, 1 - Occupied
global_map = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,0, 0],
                       [0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 0],
                       [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 1],
                       [0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 1],
                       [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,1, 1],
                       [0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,1, 0],
                       [0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 1, 1,1, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,0, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1,1, 0],
                       [0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 1, 1,1, 0],
                       [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 1, 1,1, 1]])


print('Loaded Map of Shpae - ',global_map.shape)
final_path = None
laser_ranges = np.zeros((361,))
laser_intensities = np.zeros((361,))
robot_start_location = [-8,-2,0]
robot_location = robot_start_location
robot_rotation = [0,0,0]
goal_reached = False
iter_break = False


final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]
##################### Perception Part #####################

def goal_location_marker(final_goal_location):
    '''
    Publishes the Goal Location Text in RVIZ
    '''
    marker_pub = rospy.Publisher('goal_location_marker', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.TEXT_VIEW_FACING
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'


    marker_data.scale.z = 1 # Height

    marker_data.pose.position.x = final_goal_location[0]
    marker_data.pose.position.y = final_goal_location[1]


    marker_data.color.a = 1
    marker_data.color.r = 1
    marker_data.color.g = 1
    marker_data.color.b = 1
    p = final_goal_location
    marker_data.text = 'GOAL'

    # for p in points_list:
    #     marker_data.points.append(Point(p[0],p[1],p[2]))
    marker_pub.publish(marker_data)

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
    global robot_location, robot_rotation, robot_orientation, global_map
    location = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    robot_location = location
    orientation = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation)
    rot = [roll, pitch, yaw]
    robot_rotation = rot
    robot_orientation = orientation
    robot_cube_publisher(location,robot_orientation)
    map_g = np.copy(global_map)
    OccupancyGrid_publish(map_g[::-1]) # Publish the Occupancy Grid
    goal_location_marker(final_goal_location) # Publish Goal 

def robot_cube_publisher(trans,rot):
    '''
    Publishes Robot Marker in RVIZ
    '''
    marker_pub = rospy.Publisher('robot_marker', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.CUBE
    marker_data.pose.position.x = trans[0]
    marker_data.pose.position.y = trans[1]
    qot = rot
    marker_data.pose.orientation.x = qot[0]
    marker_data.pose.orientation.y = qot[1]
    marker_data.pose.orientation.z = qot[2]
    marker_data.pose.orientation.w = qot[3]
    marker_data.header.frame_id = '/odom'
    marker_data.scale.x = 0.35
    marker_data.scale.y = 0.35
    marker_data.scale.z = 0.25
    marker_data.color.a = 1
    marker_data.color.r = 0
    marker_data.color.g = 0
    marker_data.color.b = 255
    marker_pub.publish(marker_data)

def OccupancyGrid_publish(global_map):
    '''
    Publishes Occcupancy Grid in RVIZ
    '''
    data = np.array(100*global_map,dtype=np.int8)
    map_msg = OccupancyGrid()
    map_msg.header.frame_id = '/odom'
    map_msg.info = MapMetaData()
    map_msg.info.resolution = 1
    pose_msg = Pose()
    pose_msg.position.x = -9-0.5
    pose_msg.position.y = -10-0.5
    pose_msg.orientation.x = 0
    map_msg.info.origin = pose_msg
    map_msg.info.height = data.shape[0]
    map_msg.info.width = data.shape[1]
    map_msg.data = data.ravel()
    grid_publisher = rospy.Publisher('map', OccupancyGrid,queue_size=1) # Publish Occupancy Grid to RVIZ
    grid_publisher.publish(map_msg)

def points_publisher(points_list):
    marker_pub = rospy.Publisher('path_points', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.LINE_STRIP
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/odom'

    marker_data.scale.x = 0.5 # width
    # marker_data.scale.y = 1 # Height

    marker_data.color.a = 0.6
    marker_data.color.r = 0
    marker_data.color.g = 1
    marker_data.color.b = 0

    for p in points_list:
        marker_data.points.append(Point(p[0],p[1],0))
    marker_pub.publish(marker_data)

#################### Planning Part #########################

def A_STAR(global_map, start, end, Type = '8c',e = 1, heuristic = 'eu' ):
    """
    Implements A* Algorithm for the Given Grid Map
    Arguments:
    global_map - np.array()
    start - start location [x,y]
    end - end location [x,y]
    Type - Type of Neibhours ('8c' or '4c')
    e - Heurestic Factor
    heuristic - Distance Method ('d' - distance, 'eu' - euclidean distance, 'manhattan' - manhattan distance)
    Refference - https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
    """
    print('Generating New Path')
    start_node = Node(start) # Initiate Start Node
    end_node = Node(end) # Initiate End Node

    # Check the if Start and End are iside the Map bound
    if (start_node.location[1] >   global_map.shape[0]-1 or start_node.location[1] < 0 or start_node.location[0] > global_map.shape[1]-1 or start_node.location[0] < 0):
        sys.exit('[ERROR] Start Location out of bound')
    if (end_node.location[1] >   global_map.shape[0]-1 or end_node.location[1] < 0 or end_node.location[0] > global_map.shape[1]-1 or end_node.location[0] < 0):
        sys.exit('[ERROR] End Location out of bound')

    open_list = [start_node] # Initiate Open List
    closed_nodes = [] #Initiate Closed List
    iterations = 0

    while open_list:
        current_node = open_list[0]
        index = 0
        iterations = iterations + 1

        # Try Algo with 4 Neighbours if iterations exceed 4k
        if(iterations>4000 and Type=='8c'): 
            print('Trying with 4 Neibhours')
            return None

        # Check if current Node cost is lowest
        for i, x in enumerate(open_list):
            if x.f_cost < current_node.f_cost:
                current_node = x
                index = i
        open_list.pop(index)
        closed_nodes.append(current_node)

        # Check if goal node is reached
        if current_node == end_node:
            path = []
            node = current_node
            while node is not None:
                path.append(node.location)
                node = node.prev
            # Return Back tracked path from end to goal
            return path[::-1]


        neibhours = []
        # Index changes for 8 Neibhour method
        i_list = [0,0,-1,1,-1,-1,1,1]
        j_list = [-1,1,0,0,-1,1,-1,1]
        # Index changes for 4 Neibhour method
        if(Type=='4c'):
            i_list = [0,0,1,-1]
            j_list = [1,-1,0,0]

        # Check for all Neibhours of Current Node
        for k in range(len(i_list)):

            node_pos = [current_node.location[0]+i_list[k],current_node.location[1]+j_list[k]]
            # Map Bound Check
            if (node_pos[1] >   global_map.shape[0]-1 or node_pos[1] < 0 or node_pos[0] > global_map.shape[1]-1 or node_pos[0] < 0):
                continue
            # Occupation in Grid Check
            if global_map[node_pos[1]][node_pos[0]] == 1:
                continue

            # Diagonal/Corner Cutting Check
            try:
                # if(abs(Distance_compute([node_pos[0],node_pos[1]],current_node.location,'d') - 1.4143)<0.001 and 
                #     global_map[node_pos[1]][node_pos[0]-1] == 1 and 
                #     global_map[node_pos[1]-1][node_pos[0]] == 1):
                #     continue
                # if(abs(Distance_compute([node_pos[0],node_pos[1]],current_node.location,'d') - 1.4143)<0.001 and 
                #     global_map[node_pos[1]][node_pos[0]+1] == 1 and 
                #     global_map[node_pos[1]+1][node_pos[0]] == 1):
                #     continue
                if(abs(Distance_compute([node_pos[0],node_pos[1]],current_node.location,'d') - 1.4143)<0.001 and 
                    global_map[current_node.location[1]+1][current_node.location[0]] == 1 and 
                    global_map[current_node.location[1]][current_node.location[0]+1] == 1):
                    continue
                if(abs(Distance_compute([node_pos[0],node_pos[1]],current_node.location,'d') - 1.4143)<0.001 and 
                    global_map[current_node.location[1]-1][current_node.location[0]] == 1 and 
                    global_map[current_node.location[1]][current_node.location[0]-1] == 1):
                    continue
            except IndexError:
                continue

            neibhour_node = Node((node_pos[0], node_pos[1]))
            neibhour_node.prev = current_node # Update Neibhour Node Parent
            neibhours.append(neibhour_node) # Add to other Neibhours

        for neibhour in neibhours:
            if neibhour in closed_nodes:
                continue
            
            # Avoid Locations Cornering with obstacles  
            comb_nw = [(-1,0),(-1,1),(0,1)]
            comb_ne = [(1,1),(1,0),(0,1)]
            comb_sw = [(-1,0),(-1,-1),(0,-1)]
            comb_se = [(1,-1),(1,0),(0,-1)]
            n_fac = 0
            nw_flag = True
            ne_flag = True
            sw_flag = True
            se_flag = True
            try:
                for xp,yp in comb_nw:
                    if(global_map[neibhour.location[1]+yp][neibhour.location[0]+xp]!=1):
                        nw_flag = False
                
                for xp,yp in comb_ne:
                    if(global_map[neibhour.location[1]+yp][neibhour.location[0]+xp]!=1):
                        ne_flag = False
                for xp,yp in comb_se:
                    if(global_map[neibhour.location[1]+yp][neibhour.location[0]+xp]!=1):
                        se_flag = False
                for xp,yp in comb_sw:
                    if(global_map[neibhour.location[1]+yp][neibhour.location[0]+xp]!=1):
                        sw_flag = False
                if(nw_flag == True):
                    n_fac = n_fac+3
                if(ne_flag == True):
                    n_fac = n_fac+3
                if(se_flag == True):
                    n_fac = n_fac+3
                if(sw_flag == True):
                    n_fac = n_fac+3
            except IndexError:
                pass

            # Update Costs of the Neibhour Nodes
            g = neibhour.g_cost + 1 + (n_fac**2)*10
            # g = Distance_compute(neibhour.location,start_node.location,heuristic)+n_fac
            h = Distance_compute(neibhour.location,end_node.location,heuristic)
            neibhour.update_cost(g,h*e)
            for onode in open_list:
                if neibhour == onode and neibhour.g_cost > onode.g_cost:
                    continue
            open_list.append(neibhour)
    
def convert_path(path,trans,t):
    '''
    Translates and Rotates the given set of coordinates
    '''
    npath = []
    for x in path:
        mat = [x[0],x[1]]
        mat = rot2d(mat,t)
        npath.append((mat[0]+trans[0],mat[1]+trans[1]))
    return npath

def Distance_compute(pos1,pos2,Type = 'd'):
    '''
    Distance Compute between two positions
    '''
    x1 = pos1[0]
    y1 = pos1[1]
    x2 = pos2[0]
    y2 = pos2[1]
    d = ((x1-x2)**2) + ((y1-y2)**2)
    if Type == 'd':
        return math.sqrt(d)
    if Type == 'eu':
        return d
    if Type == 'manhattan':
        return abs(x1-x2)+abs(y1-y2)

def rot2d(v,t):
    '''
    2D Rotation points
    '''
    x,y = v[0],v[1]
    xr = x*cos(t)-y*sin(t)
    yr = x*sin(t)+y*cos(t)
    return [xr,yr]

################### Motion Controll ########################

def go_to_goal(goal):
    '''
    Function to command robot in ROS stage to go to given goal wrt /odom frame
    '''
    global robot_rotation, robot_location
    d = Distance_compute(robot_location,goal)
    theta = robot_rotation[2]
    kl = 1
    ka = 4
    vx = 0
    va = 0
    heading = math.atan2(goal[1]-robot_location[1],goal[0]-robot_location[0])
    err_theta = heading - theta
    if(d>0.01):
        vx = kl*abs(d)
        vx = 1
    if(abs(err_theta)>0.01):
        va = ka*(err_theta)

    vel_1 = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist,queue_size=10) # Publish Command to robot_1
    cmd = geometry_msgs.msg.Twist()
    cmd.linear.x = vx
    cmd.angular.z = va
    vel_1.publish(cmd)

def Follow_path(path):
    '''
    Follows a given set of path
    - Reaches all the points in a list in consecutive order
    '''
    global final_goal_location, goal_reached
    cpath = path
    goal_point = cpath[-1]
    print('Following Path -->',cpath)
    for loc in cpath:
        while(Distance_compute(robot_location,loc)>0.1):
            goal_location_marker(final_goal_location)
            points_publisher(cpath)
            go_to_goal(loc)
            if(loc==goal_point):
                goal_reached = True
            

if __name__ == '__main__':
    rospy.init_node('A_STAR')
    rate = rospy.Rate(10.0)
    # Subscribe to /odom and laser sensor
    sub = rospy.Subscriber('/base_scan', LaserScan, callback_laser) # Receive Laser Msg from /stage
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom) # Receive Odom readings
    start = (int(robot_location[0]+9),int(robot_location[1]+10))
    final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]
    end = (int(final_goal_location[0]+9), int(final_goal_location[1]+10))
    # global_map[13][8] = 1 

    # A star factors
    neibhour_type = '8c'
    heuristic = 'eu'
    heuristic_factor = 2
    
    # Get the Path in Map coordinates
    final_path = A_STAR(global_map[::-1],start,end,neibhour_type,heuristic_factor,heuristic)
    if(final_path==None):
        neibhour_type = '4c'
        final_path = A_STAR(global_map[::-1],start,end,neibhour_type,heuristic_factor,heuristic)
    odom_trans = [-9,-10]
    # Convert points in Map coordinate to Global Coordinates
    path_odom_frame = convert_path(final_path,odom_trans,0)
    goal_reached = False
    
    while not rospy.is_shutdown():
        # Accuire new path when the Goal parameters are changed
        if(final_goal_location != [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]):
            final_goal_location = [rospy.get_param('/goalx'), rospy.get_param('/goaly'), 0]
            start = (int(robot_location[0]+9),int(robot_location[1]+10))
            end = (int(final_goal_location[0]+9), int(final_goal_location[1]+10))
            goal_reached = False
            final_path = A_STAR(global_map[::-1],start,end,neibhour_type,heuristic_factor,heuristic)
            path_odom_frame = convert_path(final_path,odom_trans,0)

        # Visualize the Path in RVIZ
        points_publisher(path_odom_frame)
        goal_location_marker(final_goal_location)
        if(goal_reached==False):
            # Follow the path till goal is reached
            Follow_path(path_odom_frame)
        elif(goal_reached==True):
            print('Goal Reached')
