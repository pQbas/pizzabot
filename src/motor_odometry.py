#!/usr/bin/env python3
import math
from math import sin, cos, pi

import argparse
import rospy
from std_msgs.msg import String, Float32

from nav_msgs.msg import Odometry

import tf

from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from geometry_msgs.msg import Twist
import time


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#parser = argparse.ArgumentParser(description="Implementation of LABS5 in python")
#parser.add_argument('--method',type=str , help='discret: to use discret method | ztrans: to use z-transform method| trapez: to use trapezoidal method')
#args = parser.parse_args()


d = 0.17 + 0.02 + 0.02 + 0.01
r = 0.03


rpm_right_wheel = 0.0
rpm_left_wheel = 0.0


vx = 0.0
vy = 0.0
vth = 0.0
dt = 0.0
vx_k = 0.0
vx_k1 = 0.0
vth_k = 0.0
vth_k1 = 0.0
th_k = 0.0
th_k1 = 0.0
x_k = 0.0
x_k1 = 0.0
y_k = 0.0
y_k1 = 0.0
t1 = 0.0
t0 = 0.0
Ts = 0.1


path_trapez = []
path_ztrans = []
path_discret = []


def exit():
    with open('deadreckoning_ztrans.txt', 'w') as f:
        for line in path_ztrans:
            #print(line)
            f.write(line[0])
            f.write('\n')
    with open('deadreckoning_trapez.txt','w') as f:
        for line in path_trapez:
            f.write(line[0])
            f.write('\n')
    
    with open('deadreckoning_discret.txt','w') as f:
        for line in path_discret:
            f.write(line[0])
            f.write('\n')
    print("shutdown!!!")




def callback_left(data_left):
    global rpm_left_wheel
    rpm_left_wheel = data_left.data
    
def callback_right(data_right):
    global rpm_right_wheel
    rpm_right_wheel = data_right.data


# def listener(method):
#     global vx, vy, vth
#     global dt, t1, t0
#     global th_k, th_k1, x_k, x_k1, y_k, y_k1, vx_k, vx_k1
#     t1 = rospy.get_rostime()
#     dt = t1.to_sec() - t0.to_sec()
    
#     vth_k = vth
#     vx_k = vx
#     # 1) metodo de discretizacion
#     if method == 'discret':
#         th_k = vth*dt + th_k1
#         x_k = vx*np.cos(th_k)*dt + x_k1
#         y_k = vx*np.sin(th_k)*dt + y_k1
        
#         #if method == 'all':
#         #path_discret.append([str(x_k) + " " + str(y_k)])
#         print(x_k,y_k)
#     # 2) metodo por transformada z
#     #if method == 'ztransf':
#     if (t1.to_sec() >= t0.to_sec() + Ts) and (method == 'ztransf'):
#         th_k = vth*dt + th_k1
#         x_k = vx*np.cos(th_k)*dt + x_k1
#         y_k = vx*np.sin(th_k)*dt + y_k1
#         print(x_k,y_k)
        
#         y_k1 = y_k
#         x_k1 = x_k
#         th_k1 = th_k
#         vx_k1 = vx_k
#         t0 = t1
        
        
#     # 3) regla de trapeci
#     if method == 'trapez':
#         th_k = (vth_k + vth_k1)*dt/2 + th_k
#         x_k = (vx_k*np.cos(th_k) + vx_k1*np.cos(th_k1))*dt/2 + x_k1
#         y_k = (vx_k*np.sin(th_k) + vx_k1*np.sin(th_k1))*dt/2 + y_k1
#         print(x_k,y_k)
#         #path_trapez.append([str(x_k) + " " + str(y_k)])
    
#     if method != 'ztransf':
#         y_k1 = y_k
#         x_k1 = x_k
#         th_k1 = th_k
#         vx_k1 = vx_k
#         t0 = t1
    





if __name__=='__main__': 


    rospy.init_node('motor_odometry', anonymous=True)
    rospy.Subscriber("/rpm_left_wheel_rx", Float32, callback_left, queue_size=1)
    rospy.Subscriber("/rpm_right_wheel_rx", Float32, callback_right, queue_size=1)
    
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()

    
    rospy.on_shutdown(exit)
    t1 = rospy.get_rostime()
    t0 = rospy.get_rostime()

    while not rospy.is_shutdown():

        v_right_wheel = r*2*np.pi*rpm_right_wheel/60
        v_left_wheel = r*2*np.pi*rpm_left_wheel/60

        #vx = (v_right_wheel + v_left_wheel)/2
        vx = (v_right_wheel + v_left_wheel)
        #vth = (v_right_wheel - v_left_wheel)/d
        vth = (2/d)*(v_right_wheel - v_left_wheel)
        


        #print("right_wheel:",rpm_right_wheel,"left_wheel",rpm_left_wheel)        
        #print("v_right_whee:",v_right_wheel,"v_left_wheel:",v_left_wheel)
        #print("angle_velocity:",vth)
        

        t1 = rospy.get_rostime()
        dt = t1.to_sec() - t0.to_sec()
        
        vth_k = vth
        vx_k = vx
        
        if th_k > 2*np.pi: th_k = th_k%(2*np.pi)
        
        if th_k < 0: th_k = 2*np.pi - th_k
        
        
        if (t1.to_sec() >= t0.to_sec() + Ts):

            current_time = rospy.Time.now()

            
            th_k = vth_k*dt + th_k1
            x_k = vx_k*np.cos(th_k)*dt + x_k1
            y_k = vx_k*np.sin(th_k)*dt + y_k1


            #print(x_k, y_k)


             # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th_k)
            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x_k, y_k, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )
            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            # set the position
            odom.pose.pose = Pose(Point(x_k, y_k, 0.), Quaternion(*odom_quat))
            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx_k*np.cos(th_k), vx_k*np.sin(th_k), 0), Vector3(0, 0, vth_k))
            # publish the message
            odom_pub.publish(odom)

            
            y_k1 = y_k
            x_k1 = x_k
            th_k1 = th_k
            vx_k1 = vx_k
            t0 = t1


