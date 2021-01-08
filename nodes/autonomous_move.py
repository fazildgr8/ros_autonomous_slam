#!/usr/bin/python

from rrt import find_path_RRT
import numpy as np
from scipy.misc import imread
import math
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, GridCells
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import actionlib
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import cv2
from math import cos,sin
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global final_goal_location, goal_reached, robot_location,robot_rotation, current_map
final_goal_location = [326.0, 279.0]
goal_reached = False
robot_rotation = [0,0,0]
robot_location = [-2,0.5,0]
current_map = np.zeros((384,384))


def movebase_client(x,y):
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

# if __name__ == '__main__':
#     try:
#         rospy.init_node('movebase_client_py')
#         result = movebase_client(-2,3)
#         if result:
#             rospy.loginfo("Goal execution done!")
#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation test finished.")

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

def convert_point(x,trans,t):
    '''
    Translates and Rotates the given set of coordinate
    '''
    mat = [x[0],x[1]]
    mat = rot2d(mat,t)
    return (mat[0]+trans[0],mat[1]+trans[1])

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

def go_to_goal(goal):
    '''
    Function to command robot in ROS stage to go to given goal wrt /odom frame
    '''
    global robot_location,robot_rotation
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
            # goal_location_marker(final_goal_location)
            # points_publisher(cpath)
            
            go_to_goal([loc[0]/10,loc[1]/10])
            if(loc==goal_point):
                goal_reached = True

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

def callback_map(msg):
    global current_map
    data = np.array(msg.data)
    map_width = msg.info.width
    map_height = msg.info.height
    current_map =  np.reshape(data,(map_height,map_width))

def map_img(arr):
    disp_map = np.ones((384,384))*255
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            if arr[i][j]==-1:
                disp_map[i][j] = 100
            if arr[i][j] == 100:
                disp_map[i][j] = 0
    im = np.array(disp_map, dtype = np.uint8)
    return im[::-1]

def points_publisher(points_list):
    marker_pub = rospy.Publisher('path_points', Marker,queue_size=1) # Publish Robot Position to RVIZ
    marker_data = Marker()
    marker_data.type = marker_data.POINTS
    marker_data.action = marker_data.ADD
    marker_data.header.frame_id = '/map'

    marker_data.scale.x = 0.1 # width
    marker_data.scale.y = 0.1 # Height

    marker_data.color.a = 1
    marker_data.color.r = 1
    marker_data.color.g = 0
    marker_data.color.b = 0

    for p in points_list:
        marker_data.points.append(Point(p[0],p[1],0))
    marker_pub.publish(marker_data)

if __name__ == '__main__':
    rospy.init_node('RRT_Explorer')
    rate = rospy.Rate(10.0)
    # Subscribe to /odom
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odom) # Receive Odom readings
    sub_map = rospy.Subscriber('/map',OccupancyGrid, callback_map) # Receive Map
    img = imread('/home/fazildgr8/catkin_ws/src/ros_autonomous_slam/media/my_map.png')
    
    flag = True
    while not rospy.is_shutdown():
        start, goal = (convert_point(robot_location,[192,192],0), convert_point([2,0.5],[192,192],0))
        print(start,goal)
        if flag:
            path,graph = find_path_RRT(convert_point(robot_location,[192,192],0),convert_point([-1,0.5],[192,192],0),cv2.cvtColor(map_img(current_map), cv2.COLOR_GRAY2BGR)[::-1])
            print(path)
            print(graph)
            flag = False
        print('Map Shape',current_map.shape)
        # points_publisher(convert_path(path,[-192,-192],0))
        # if flag:
        #     result = movebase_client(-2,0.5)
        #     if result:
        #         flag = False
        #         print('reached')
        cv2.imshow('Constructed Map',map_img(current_map))
        # np.savetxt('my_map',map_img(current_map))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()


