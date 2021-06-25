#!/usr/bin/env python
from pynput.keyboard import Key, Listener
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

angular_z=0;
V=.4;
rospy.init_node("speed_controller")
pub_vel = rospy.Publisher("/cmd_vel", Twist , queue_size = 1)
pub_fl = rospy.Publisher("/simple_model/base_to_fl_steer_joint_position_controller/command", Float64 , queue_size = 1)
pub_fr = rospy.Publisher("/simple_model/base_to_fr_steer_joint_position_controller/command", Float64 , queue_size = 1)
speed = Twist()
while not rospy.is_shutdown():
	def on_press(key):
		if key == key.up:
		    print 'go forword'
		    speed.linear.x=V;
		    pub_vel.publish(speed)
		if key == key.left:
		    print 'go to left'
		    angular_z=.7;
		    pub_fl.publish(angular_z)
		    pub_fr.publish(angular_z)
		if key == key.right:
		    print 'go to right'
		    angular_z=-.7;
		    pub_fl.publish(angular_z)
		    pub_fr.publish(angular_z)
		if (key != key.left and key != key.right):
		    angular_z=0;
		    pub_fl.publish(angular_z)
		    pub_fr.publish(angular_z)
		if key == key.down:
		    print 'go back'
		    speed.linear.x=-V;
		    pub_vel.publish(speed)
		if key == key.space:
		    print 'Stop'
		    speed.linear.x=0;
		    pub_vel.publish(speed)
		print key
	def on_release(key):
		if key == Key.esc:
		    return False
		    
	# Collect events until released
	with Listener(
		    on_press=on_press,
		    on_release=on_release) as listener:
		listener.join()
		
		
