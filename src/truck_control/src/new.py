#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64

rospy.init_node("speed_controller")
pub_vel = rospy.Publisher("/cmd_vel", Twist , queue_size = 1)
pub_fl = rospy.Publisher("/simple_model/base_to_fl_steer_joint_position_controller/command", Float64 , queue_size = 1)
pub_fr = rospy.Publisher("/simple_model/base_to_fr_steer_joint_position_controller/command", Float64 , queue_size = 1)

while not rospy.is_shutdown():
    speed = Twist()  
    V = 0.1 #m/s
    speed.linear.x  = V
    pub_vel.publish(speed)
    ## 
    angz = 0.3 ## rad
    pub_fl.publish(angz)
    pub_fr.publish(angz)
