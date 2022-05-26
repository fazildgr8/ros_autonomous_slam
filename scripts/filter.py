#!/usr/bin/env python3

# --------Include modules---------------
from copy import copy
import rclpy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf2_ros
from numpy import array, vstack, delete
from ros_autonomous_slam.functions import gridValue, informationGain
from sklearn.cluster import MeanShift
from ros_autonomous_slam.msg import PointArray

# Subscribers' callbacks------------------------------
mapData = OccupancyGrid()
frontiers = []
globalmaps = []


def callBack(data, args):
    global frontiers, min_distance
    transformedPoint = args[0].transformPoint(args[1], data)
    x = [array([transformedPoint.point.x, transformedPoint.point.y])]
    if len(frontiers) > 0:
        frontiers = vstack((frontiers, x))
    else:
        frontiers = x


def mapCallBack(data):
    global mapData
    mapData = data


def globalMap(data):
    global global1, globalmaps, litraIndx, namespace_init_count, n_robots
    global1 = data
    if n_robots > 1:
        indx = int(data._connection_header['topic']
                   [litraIndx])-namespace_init_count
    elif n_robots == 1:
        indx = 0
    globalmaps[indx] = data

# Node----------------------------------------------


def node():
    global frontiers, mapData, global1, global2, global3, globalmaps, litraIndx, n_robots, namespace_init_count
    rclpy.init_node('filter', anonymous=False)

    # fetching all parameters
    map_topic = rclpy.get_param('~map_topic', 'map')
    threshold = rclpy.get_param('~costmap_clearing_threshold', 70)
    # this can be smaller than the laser scanner range, >> smaller >>less computation time>> too small is not good, info gain won't be accurate
    info_radius = rclpy.get_param('~info_radius', 1.0)
    goals_topic = rclpy.get_param('~goals_topic', '/detected_points')
    n_robots = rclpy.get_param('~n_robots', 1)
    namespace = rclpy.get_param('~namespace', '')
    namespace_init_count = rclpy.get_param('namespace_init_count', 1)
    rateHz = rclpy.get_param('~rate', 100)
    global_costmap_topic = rclpy.get_param(
        '~global_costmap_topic', '/move_base/global_costmap/costmap')
    robot_frame = rclpy.get_param('~robot_frame', 'base_link')

    litraIndx = len(namespace)
    rate = rclpy.Rate(rateHz)
# -------------------------------------------
    rclpy.Subscriber(map_topic, OccupancyGrid, mapCallBack)


# ---------------------------------------------------------------------------------------------------------------

    for i in range(0, n_robots):
        globalmaps.append(OccupancyGrid())

    if len(namespace) > 0:
        for i in range(0, n_robots):
            rclpy.Subscriber(namespace+str(i+namespace_init_count) +
                             global_costmap_topic, OccupancyGrid, globalMap)
    elif len(namespace) == 0:
        rclpy.Subscriber(global_costmap_topic, OccupancyGrid, globalMap)
# wait if map is not received yet
    while (len(mapData.data) < 1):
        rclpy.loginfo('Waiting for the map')
        rclpy.sleep(0.1)
        pass
# wait if any of robots' global costmap map is not received yet
    for i in range(0, n_robots):
        while (len(globalmaps[i].data) < 1):
            rclpy.loginfo('Waiting for the global costmap')
            rclpy.sleep(0.1)
            pass

    global_frame = "/"+mapData.header.frame_id

    tf2_rosLisn = tf2_ros.TransformListener()
    if len(namespace) > 0:
        for i in range(0, n_robots):
            tf2_rosLisn.waitf2_rosorTransform(global_frame[1:], namespace+str(
                i+namespace_init_count)+'/'+robot_frame, rclpy.Time(0), rclpy.Duration(10.0))
    elif len(namespace) == 0:
        tf2_rosLisn.waitf2_rosorTransform(
            global_frame[1:], '/'+robot_frame, rclpy.Time(0), rclpy.Duration(10.0))

    rclpy.Subscriber(goals_topic, PointStamped, callback=callBack,
                     callback_args=[tf2_rosLisn, global_frame[1:]])
    pub = rclpy.Publisher('frontiers', Marker, queue_size=10)
    pub2 = rclpy.Publisher('centroids', Marker, queue_size=10)
    filterpub = rclpy.Publisher('filtered_points', PointArray, queue_size=10)

    rclpy.loginfo("the map and global costmaps are received")

    # wait if no frontier is received yet
    while len(frontiers) < 1:
        pass

    points = Marker()
    points_clust = Marker()
# Set the frame ID and timestamp.  See the tf2_ros tutorials for information on these.
    points.header.frame_id = mapData.header.frame_id
    points.header.stamp = rclpy.Time.now()

    points.ns = "markers2"
    points.id = 0

    points.type = Marker.POINTS

# Set the marker action for latched frontiers.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points.action = Marker.ADD

    points.pose.orientation.w = 1.0

    points.scale.x = 0.2
    points.scale.y = 0.2

    points.color.r = 255.0/255.0
    points.color.g = 255.0/255.0
    points.color.b = 0.0/255.0

    points.color.a = 1
    points.lifetime = rclpy.Duration()

    p = Point()

    p.z = 0

    pp = []
    pl = []

    points_clust.header.frame_id = mapData.header.frame_id
    points_clust.header.stamp = rclpy.Time.now()

    points_clust.ns = "markers3"
    points_clust.id = 4

    points_clust.type = Marker.POINTS

# Set the marker action for centroids.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    points_clust.action = Marker.ADD

    points_clust.pose.orientation.w = 1.0

    points_clust.scale.x = 0.2
    points_clust.scale.y = 0.2
    points_clust.color.r = 0.0/255.0
    points_clust.color.g = 255.0/255.0
    points_clust.color.b = 0.0/255.0

    points_clust.color.a = 1
    points_clust.lifetime = rclpy.Duration()

    temppoint = PointStamped()
    temppoint.header.frame_id = mapData.header.frame_id
    temppoint.header.stamp = rclpy.Time(0)
    temppoint.point.z = 0.0

    arraypoints = PointArray()
    tempPoint = Point()
    tempPoint.z = 0.0
# -------------------------------------------------------------------------
# ---------------------     Main   Loop     -------------------------------
# -------------------------------------------------------------------------
    while not rclpy.is_shutdown():
        # -------------------------------------------------------------------------
        # Clustering frontier points
        centroids = []
        front = copy(frontiers)
        if len(front) > 1:
            ms = MeanShift(bandwidth=0.3)
            ms.fit(front)
            centroids = ms.cluster_centers_  # centroids array is the centers of each cluster

        # if there is only one frontier no need for clustering, i.e. centroids=frontiers
        if len(front) == 1:
            centroids = front
        frontiers = copy(centroids)
# -------------------------------------------------------------------------
# clearing old frontiers

        z = 0
        while z < len(centroids):
            cond = False
            temppoint.point.x = centroids[z][0]
            temppoint.point.y = centroids[z][1]

            for i in range(0, n_robots):

                transformedPoint = tf2_rosLisn.transformPoint(
                    globalmaps[i].header.frame_id, temppoint)
                x = array([transformedPoint.point.x, transformedPoint.point.y])
                cond = (gridValue(globalmaps[i], x) > threshold) or cond
            if (cond or (informationGain(mapData, [centroids[z][0], centroids[z][1]], info_radius*0.5)) < 0.2):
                centroids = delete(centroids, (z), axis=0)
                z = z-1
            z += 1
# -------------------------------------------------------------------------
# publishing
        arraypoints.points = []
        for i in centroids:
            tempPoint.x = i[0]
            tempPoint.y = i[1]
            arraypoints.points.append(copy(tempPoint))
        filterpub.publish(arraypoints)
        pp = []
        for q in range(0, len(frontiers)):
            p.x = frontiers[q][0]
            p.y = frontiers[q][1]
            pp.append(copy(p))
        points.points = pp
        pp = []
        for q in range(0, len(centroids)):
            p.x = centroids[q][0]
            p.y = centroids[q][1]
            pp.append(copy(p))
        points_clust.points = pp
        pub.publish(points)
        pub2.publish(points_clust)
        rate.sleep()
# -------------------------------------------------------------------------


if __name__ == '__main__':
    try:
        node()
    except rclpy.ROSInterruptException:
        pass
