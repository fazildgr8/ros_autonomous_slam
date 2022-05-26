#!/usr/bin/env python


#--------Include modules---------------
import rclpy
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from ros_autonomous_slam.getfrontier import getfrontier

#-----------------------------------------------------
# Subscribers' callbacks------------------------------
mapData=OccupancyGrid()


def mapCallBack(data):
    global mapData
    mapData=data
    

    

# Node----------------------------------------------
def node():
		global mapData
		exploration_goal=PointStamped()
		rclpy.init_node('detector', anonymous=False)
		map_topic= rclpy.get_param('~map_topic','/map')
		rclpy.Subscriber(map_topic, OccupancyGrid, mapCallBack)
		targetspub = rclpy.Publisher('/detected_points', PointStamped, queue_size=10)
		pub = rclpy.Publisher('shapes', Marker, queue_size=10)
# wait until map is received, when a map is received, mapData.header.seq will not be < 1
		while mapData.header.seq<1 or len(mapData.data)<1:
			pass
    	   	
		rate = rclpy.Rate(50)	
		points=Marker()

		#Set the frame ID and timestamp.  See the TF tutorials for information on these.
		points.header.frame_id=mapData.header.frame_id
		points.header.stamp=rclpy.Time.now()

		points.ns= "markers"
		points.id = 0

		points.type = Marker.POINTS
		#Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
		points.action = Marker.ADD;

		points.pose.orientation.w = 1.0;
		points.scale.x=points.scale.y=0.3;
		points.color.r = 255.0/255.0
		points.color.g = 0.0/255.0
		points.color.b = 0.0/255.0
		points.color.a=1;
		points.lifetime == rclpy.Duration();

#-------------------------------OpenCV frontier detection------------------------------------------
		while not rclpy.is_shutdown():
			frontiers=getfrontier(mapData)
			for i in range(len(frontiers)):
				x=frontiers[i]
				exploration_goal.header.frame_id= mapData.header.frame_id
				exploration_goal.header.stamp=rclpy.Time(0)
				exploration_goal.point.x=x[0]
				exploration_goal.point.y=x[1]
				exploration_goal.point.z=0	


				targetspub.publish(exploration_goal)
				points.points=[exploration_goal.point]
				pub.publish(points) 
			rate.sleep()
          	
		

	  	#rate.sleep()



#_____________________________________________________________________________

if __name__ == '__main__':
    try:
        node()
    except rclpy.ROSInterruptException:
        pass
 
 
 
 
