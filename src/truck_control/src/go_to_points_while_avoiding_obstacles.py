#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from operator import itemgetter
import matplotlib.pyplot as plt
from itertools import *
import sensor_msgs.msg
from math import atan2
import numpy as np
import random
import time
import math

LINX = 0.0 #Always forward linear velocity.
THRESHOLD = 1.5 #THRESHOLD value for laser scan.
PI = math.pi
data_ban=-1
data0_ban=1
Kp =.025
angz = 0.1
x = 0.0
y = 0.0 
theta = 0.0
yaw=0
V=1
X=np.array([])
Y=np.array([])
ang=np.array([])
Yaw=np.array([])
def imu_data0(msg):
    global yaw
    yaw = msg.orientation.z

def newOdom(msg):
    global x
    global y
    global theta
    x = 1.1*msg.pose.pose.position.x
    y = 1.1*msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
rospy.init_node("speed_controller")
sub = rospy.Subscriber("/imu_data", Imu,imu_data0)
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub_vel = rospy.Publisher("/cmd_vel", Twist , queue_size = 1)
pub_ban = rospy.Publisher("/simple_model/base_to_ban_joint_position_controller/command", Float64 , queue_size = 1)
pub_fl = rospy.Publisher("/simple_model/base_to_fl_steer_joint_position_controller/command", Float64 , queue_size = 1)
pub_fr = rospy.Publisher("/simple_model/base_to_fr_steer_joint_position_controller/command", Float64 , queue_size = 1)
r = rospy.Rate(3)
goal = Point()
goal.x  = 8
goal.y  = 8
i=1

while not rospy.is_shutdown():
	speed = Twist()
	inc_x = goal.x -x
	inc_y = goal.y -y
	speed = Twist()

	angle_to_goal = atan2(inc_y, inc_x)
	alpha = angle_to_goal - theta
#### command
	if (abs(alpha) > PI/2):
		angular_z = 0.5
		speed.linear.x  =  V
		#print "yes"
	else:
		speed.linear.x  =  V
		angular_z = alpha
		
#### stop command
	if abs(inc_x)     <= 0.5:
		if abs(inc_y) <= 0.5:
			#print "inc_x",inc_x , "inc_y",inc_y ,"xg",goal.x, "yg",goal.y, "x" , x, "y" , y
			speed.linear.x  = -V
			angular_z = (abs(inc_x)+abs(inc_y))/2
			pub_vel.publish(speed)
			#print "speed.linear.x", speed.linear.x
			time.sleep(4)
			speed.linear.x  = 0
			angular_z       = 0
	def LaserScanProcess(data):
		range_angels = np.arange(len(data.ranges))
		ranges = np.array(data.ranges)
		range_mask = (ranges > THRESHOLD)
		ranges = list(range_angels[range_mask])
		max_gap = 40
		# #print(ranges)
		gap_list = []
		for k, g in groupby(enumerate(ranges), lambda (i,x):i-x):
		    gap_list.append(map(itemgetter(1), g))
		gap_list.sort(key=len)
		largest_gap = gap_list[-1]
		min_angle, max_angle = largest_gap[0]*((data.angle_increment)*180/PI), largest_gap[-1]*((data.angle_increment)*180/PI)
		average_gap = (max_angle - min_angle)/2

		turn_angle = min_angle + average_gap
		global LINX
		global angz
		#if average_gap < max_gap:
		#    angz = 0.
		if average_gap > max_gap:
		    LINX = 0.2
		    angz = Kp*(90 - turn_angle)
		if angz > 0.5:
			angz=0.5
		if angz < -0.5:
			angz = -0.5
	rospy.Subscriber("scan", sensor_msgs.msg.LaserScan , LaserScanProcess)
	if angular_z >  0.5:
		angular_z=  0.5
	if angular_z  <-0.5:
		angular_z =-0.5
	if speed.linear.x >1:
		speed.linear.x=1
	if angz < 0.1:
		pub_fl.publish(angular_z)
		pub_fr.publish(angular_z)
		pub_vel.publish(speed)
	else:
		speed = Twist()
		speed.linear.x=LINX
		speed.angular.z=0
		pub_fl.publish(-angz)
		pub_fr.publish(-angz)
		pub_vel.publish(speed)
		
	if angular_z == 0 and speed.linear.x == 0:
	    pub_ban.publish(data_ban)
	    time.sleep(3)
	    data0_ban=0
	    pub_ban.publish(data0_ban)
	    time.sleep(5)   
		
	X=np.hstack((X,x))
	Y=np.hstack((Y,y))
	Yaw=np.hstack((Yaw,yaw))
	ang = np.hstack((ang,speed.angular.z))
	if np.size(Y) == np.size(X) and data0_ban==0:
		#plt.plot(angular_z,linestyle='solid', marker='*');
		#plt.show()
		plt.plot(X,Y,linestyle='solid', marker='*'),plt.grid(True);
		plt.show()
		break		
	#print "angular_z", angular_z ,'speed.linear.x',speed.linear.x , 'angz' , angz
	time.sleep(.5)


	
