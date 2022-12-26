#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import numpy as np

d = 0.23
r = 0.03

rpm_right_wheel = 0
rpm_left_wheel = 0

position = Point(x=0,y=0,z=0)
orientation = Quaternion(x=0,y=0,z=0,w=0)
pose2 = Pose(position,orientation)
R = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0] , [np.sin(np.pi/2), 
np.cos(np.pi/2), 0], [0, 0, 1]])


def compute_rpms(data):

    global rpm_right_wheel
    global rpm_left_wheel

    vx = data.linear.x
    wz = data.angular.z

    v_right_wheel = vx + wz*d/2
    v_left_wheel = vx - wz*d/2

    rpm_right_wheel = int(v_right_wheel*60/(r*2*np.pi))
    rpm_left_wheel = int(v_left_wheel*60/(r*2*np.pi))

    rospy.loginfo(rpm_right_wheel)
    rospy.loginfo(rpm_left_wheel)


def main():
    
    right_wheel = rospy.Publisher('rpm_right_wheel_tx', Float32, queue_size = 1)
    left_wheel = rospy.Publisher('rpm_left_wheel_tx', Float32, queue_size = 1)

    rospy.init_node('motor_controller', anonymous=True)
    
    rate = rospy.Rate(100)
    rospy.Subscriber("cmd_vel", Twist, compute_rpms)

    
    while not rospy.is_shutdown():
        #rospy.loginfo(pose2)

        right_wheel.publish(rpm_right_wheel)
        left_wheel.publish(rpm_left_wheel)
        
        #print(rpm_right_wheel,rpm_left_wheel)
        rate.sleep()


if __name__ == '__main__':
    main()





