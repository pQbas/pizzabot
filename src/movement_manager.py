#!/usr/bin/env python3
import argparse
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time

from std_msgs.msg import Float32, Float64MultiArray
import tf


# parser = argparse.ArgumentParser(description="Implementation of LABS5 in python")
# parser.add_argument('--X',type=str, help='position in x-axe')
# parser.add_argument('--Y',type=str, help='position in y-axe')
# args = parser.parse_args()

Frep = [0,0]

x_g = 1.0
y_g = 0.0
th_g = np.arctan2(y_g, x_g)

th_k = 0.0
x_k = 0.0
y_k = 0.0


def exit():
    move_cmd = Twist()
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub_cmd_vel.publish(move_cmd)

def get_goal(array_position):
    global x_g
    global y_g
    global th_g

    x_g,y_g = array_position.data
    th_g = np.arctan2(y_g, x_g)


def get_pose(data):
    global x_k
    global y_k
    global th_k

    x_k = data.pose.pose.position.x
    y_k = data.pose.pose.position.y
    
    quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    th_k = yaw



def get_repulsion_force(data):
    global Frep
    Frep = [0,0]
    Frep = data.data



if __name__=='__main__': 
    rospy.init_node('movement_manager', anonymous=True)
    rospy.Subscriber("/odom", Odometry, get_pose)
    #rospy.Subscriber("/goal_position", Float64MultiArray, get_goal)
    #rospy.Subscriber("/repulsion_force", Float64MultiArray, get_repulsion_force)
    rospy.on_shutdown(exit)
    
    distance_error_pub = rospy.Publisher('/error_distance', Float32, queue_size=1)
    angular_error_pub = rospy.Publisher('/error_angular', Float32, queue_size=1)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    rate_cmd_vel = rospy.Rate(80)
    move_cmd = Twist()
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    
    while not rospy.is_shutdown():

        # calcula el error en x,y
        error_x = x_g - x_k #+ abs(Frep[0])
        error_y = y_g - y_k #+ abs(Frep[1])
        
        move_cmd = Twist()
        th_g = np.arctan2(error_y, error_x)
        err_th_k = th_g - th_k


        distance_error = np.sqrt((error_x)**2 + (error_y)**2)
        linear_velocity = 0.015 * distance_error


        # limite superior e inferior a la velocidad linear
        if linear_velocity > 0.2: 
            linear_velocity = 0.2
        elif linear_velocity < 0.1:
            linear_velocity = 0.1

        # velocidad angular constante, dependiendo del signo
        if err_th_k>0:
            angular_velocity = 0.6
        else:
            angular_velocity = -0.6

        # condicion de paro
        if distance_error < 0.1:
            move_cmd.linear.x = 0.0
        
        # condicion de movimiento
        else:
            if np.abs(err_th_k) > 0.6: # se mueve angularmente si existe error angular
                move_cmd.angular.z = angular_velocity
                move_cmd.linear.x = 0.0
            else:                      # se mueve linealmente si existe error linear
                move_cmd.linear.x = linear_velocity
                move_cmd.angular.z = 0.0
        
        distance_error_pub.publish(distance_error)
        angular_error_pub.publish(err_th_k)

        pub_cmd_vel.publish(move_cmd)

            
