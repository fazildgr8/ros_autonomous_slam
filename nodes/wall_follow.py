#!/usr/bin/env python

'''
source-
https://github.com/vipulkumbhar/AuE893_Autonomy_Science_and_Systems/blob/master/catkin_ws/src/assignment4/src/turtlebot3_wallfollowing.py
'''
import rospy
import numpy as np
from numpy import inf
from geometry_msgs.msg  import Twist, Vector3
from sensor_msgs.msg import LaserScan

side_scanstartangle = 20 
side_scanrange      = 60          
front_scanrange     = 16
distancefromwall    = 0.4

x   = np.zeros((360))
s_d = 0 # front wall distance
y_l = 0 # left wall distance
y_r = 0 # right wall distance

# PID parameters
kp = 4
kd = 450
ki = 0

k1 = kp + ki + kd
k2 = -kp - 2*kd
k3 = kp


def callback(data):
	global y_l, y_r,x,s_d,front_scanrange,side_scanstartangle,side_scanrange

	x  = list(data.ranges)
	for i in range(360):
		if x[i] == inf:
			x[i] = 7
		if x[i] == 0:
			x[i] = 6

        # store scan data 
	y_l= min(x[side_scanstartangle:side_scanstartangle+side_scanrange])          # left wall distance
	y_r= min(x[360-side_scanstartangle-side_scanrange:360-side_scanstartangle])  # right wall distance
	s_d= min(min(x[0:int(front_scanrange/2)],x[int(360-front_scanrange/2):360])) # front wall distance

def tmnt_controller():

	#Setup
	global k1,k2,k3,kp,kd,ki,s_d,x,y_r,y_l
	global distancefromwall

	# Initialize parameters used in PID controller
	prev_PID_output = 0
	prev_error      = 0
	prev_prev_error = 0

	rospy.init_node('wallfollowing_control', anonymous=True)
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	scan_subscriber    = rospy.Subscriber('/scan', LaserScan, callback)
	rate               = rospy.Rate(10)                  # 20hz

	while not rospy.is_shutdown():

		delta = distancefromwall-y_r   # distance error 

		#PID controller
		# PID_output = prev_PID_output + k1*delta + k2*prev_error + k3*prev_prev_error

		#PD controller
		PID_output  = kp*delta + kd*(delta-prev_error)

		#stored states
		prev_error      = delta
		prev_prev_error = prev_error
		prev_PID_output = PID_output

		#clip PID output
		angular_zvel = np.clip(PID_output,-1.2,1.2)

		linear_vel   = np.clip((s_d-0.35),-0.1,0.4)

		#if linear_vel <0:
		#	angular_zvel = angular_zvel*-1 

		#if s_d < distancefromwall/3 and y_l >3:			
		#	PID_output = 0.1
		
		
		#check IOs
		print('distance from right wall in cm =',format(int(y_r*100)),'/',format(distancefromwall*100))
		print('distance from front wall in cm =',format(s_d*100))
		print('linear_vel=',format(linear_vel),' angular_vel=',format(angular_zvel))
		rospy.loginfo('\n') 

		#publish cmd_vel
		vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
		velocity_publisher.publish(vel_msg)
		rate.sleep()

	velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
	print('Turtlebot stopped')

if __name__ == '__main__':
	try:
		#start turtllebot
		tmnt_controller()

	except rospy.ROSInterruptException: pass