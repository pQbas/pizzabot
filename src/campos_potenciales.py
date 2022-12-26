#!/usr/bin/env python3

#import rospy
import tf
import time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
#from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion,quaternion_from_euler
#import cv2

import rospy
from sensor_msgs.msg import LaserScan
import time
import rospy
import math as m
import numpy as np
# import cv2



# for i in range(0,len(laser_ranges)):
#     laser_angles.append(msg.angle_min + i*msg.angle_increment)

# for i in range(0, len(laser_ranges)):
#     if (math.isinf(laser_ranges[i])==False):
#         f = (1/((laser_ranges[i])**2))*math.cos(laser_angles[i])
#         s = (1/((laser_ranges[i])**2))*math.sin(laser_angles[i])
#         F=F+f
#         S=S+s
# try:
#     Xrep=Krep*F*math.cos(thetarob) - Krep*S*math.sin(thetarob)
#     Yrep=Krep*F*math.sin(thetarob) + Krep*S*math.cos(thetarob)
# except:
#     print("Warning!!!: theraob variable is not find, see the odom topic")



# def callback(msg):
#     global R
#     global thetarob
#     global Xatt
#     global Yatt
#     global image

#     print("Callback")
    
#     p = msg.pose.pose 
    
#     xrob = p.position.x
#     yrob = p.position.y
#     Xatt = xg-xrob
#     Yatt = yg-yrob

#     R = np.hypot(Xatt , Yatt)

#     orientation_q = msg.pose.pose.orientation
#     orientation_list=[orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

#     (roll,pitch,thetarob)=euler_from_quaternion(orientation_list)



#     image = np.zeros([900,900])
#     start_point = (450, 450)
#     kmin = 0.00001
#     end_point = (int(kmin*(Xatt - start_point[0])) , int(kmin*(Yatt - start_point[1])))
#     color = (255, 255, 255)
#     thickness = 5
#     image = cv2.arrowedLine(image, start_point, end_point,color,thickness)
    


# def main(R):

#     try:
#         Px=Xatt-Xrep
#         Py=Yatt-Yrep
        
#         R = np.hypot(Px,Py)
#         thetap=math.atan2(Py,Px)

        
        
#         if R > umbral:
#             Rmax = umbral
#             Patt = Katt*Rmax

            
#             #distance_error_pub.publish(distance_error)
#             #pub_cmd_vel.publish(move_cmd)
#             #print("FUERA de umbral")
                
#         else: 
#             Patt = Katt*R

            
#     except:
#         print("Warning: There are variables not recognized in main function!!!")




R = 1
vx = vth = xrob = yrob = 0
xg = 3.0
yg = 3.0
umbral = 1.8
Katt = 1
Krep = 0.001
twist = Twist()
image = np.zeros([900,900])

ranges = []
laser_angles = []


# def draw_arrow(image, magnitud, angle, K):

#     #image = np.zeros([900,900])
#     magnitud = K*magnitud
#     start_point = (450, 450)
#     end_point = (int(start_point[0] + magnitud*m.cos(angle)) , 
#                 int(start_point[1] + magnitud*m.sin(angle)))
#     color = (255, 255, 255)
#     thickness = 1
#     return cv2.arrowedLine(image, start_point, end_point,color, thickness)


# def draw_point(image, pointx, pointy, k):

#     #image = np.zeros([900,900])
#     start_point = (450, 450)
#     end_point = (int(start_point[0] + k*pointx) , 
#                 int(start_point[1] + k*pointy))

#     color = (255, 255, 255)
#     thickness = 1

#     return cv2.circle(image, end_point, 1, color, thickness)


def lidar(msg):
    # global laser_ranges
    # global laser_angles
    # global Xrep
    # global Yrep
    # laser_ranges = []
    # laser_angles = []
    # f = s = F = S = 0
    # laser_ranges = msg.ranges
    # print(msg.ranges)
    global Frep
    global image
    global ranges
    global laser_angles
    ranges = msg.ranges

    # for i in range(0,len(ranges)):
    #     laser_angles.append(msg.angle_min+i*msg.angle_increment) 
    
    image = np.zeros([900,900])
    
    # compute Frep
    Frep = []
    points_in_range = []
    rep_force = [0,0]
    for i in range(0,len(ranges)):
        if (m.isinf(ranges[i])==False):
            angle = msg.angle_min + i*msg.angle_increment
            point_pose = (ranges[i]*m.cos(angle), ranges[i]*m.sin(angle), angle)
            point_lidar_distance = m.dist( [point_pose[0],0] , [0,point_pose[1]] )
            if point_lidar_distance < 0.3:
                points_in_range.append((point_pose[0], point_pose[1]))
                rep_force_magnitude = -Krep/(ranges[i]**2)
                rep_force_components = [rep_force_magnitude*m.cos(angle), rep_force_magnitude*m.sin(angle)]   
                rep_force[0] = rep_force[0] + rep_force_components[0]
                rep_force[1] = rep_force[1] + rep_force_components[1]            
    Frep.append(rep_force)

    # compute Fatt
    Fatt = []
    goal_position = [2,2]
    goal_position_distance = m.dist([goal_position[0],0], [0,goal_position[1]])
    att_force_magnitud = Katt/(goal_position_distance**2)
    angle = m.atan2(goal_position[1],goal_position[0])
    att_force = [att_force_magnitud*m.cos(angle), att_force_magnitud*m.sin(angle)]
    Fatt.append(att_force)


    # plotting the Frep, Fatt
    # for Fx, Fy in Frep:
    #     Fmag = m.dist([Fx,0], [0,Fy])
    #     Fangle = m.atan2(Fy,Fx)
    #     image = draw_arrow(image, Fmag, Fangle, 200)

    # for Fx,Fy in Fatt:
    #     Fmag = m.dist([Fx,0], [0,Fy])
    #     Fangle = m.atan2(Fy,Fx)
    #     image = draw_arrow(image, Fmag, Fangle, 200)

    # for px, py in points_in_range:
    #     image = draw_point(image, px, py, 100)

    # cv2.imshow('arrow', image)
    # cv2.waitKey(1)


Frep = [[0,0]]

if __name__ == '__main__':

    rospy.init_node('campos_potenciales', anonymous=True)
    #rospy.Subscriber('/odom', Odometry, callback)
    rospy.Subscriber("/scan", LaserScan, lidar)
    pub_Frep = rospy.Publisher("/repulsion_force", Float64MultiArray, queue_size=1)

    data_Frep = Float64MultiArray()
    rate = rospy.Rate(80)
    

    while not rospy.is_shutdown():    
        #pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        time.sleep(0.1)
        #print(Frep)

        data_Frep.data = Frep[0]
        pub_Frep.publish(data_Frep)
        rate.sleep()
        
        # #main(R)
        # try:
        #     if R > 0.2:
        #         main(R)
        #     else:
        #         print("Llegamos")
        #         twist.linear.x=0
        #         #pub.publish(twist)
        # except:
        #     print("Warning: Esta es otra advertencia !!! ")