#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64MultiArray



if __name__ == '__main__':

    try:
        goal_pub = rospy.Publisher('goal_position', Float64MultiArray, queue_size=1)
        data_to_send = Float64MultiArray()
        array = [3,1]
        rospy.init_node('goal_publisher', anonymous=True)
        rate = rospy.Rate(80) # 10hz

        while not rospy.is_shutdown():
            data_to_send.data = array
            goal_pub.publish(data_to_send)
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass