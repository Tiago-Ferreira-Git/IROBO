#!/usr/bin/env python3
from __future__ import print_function
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

import time


def callback(data):
        print("-----------------------------------------------------------------------")
        print(data.poses)

        return data.poses


if __name__ == '__main__':
        
        rospy.init_node('performance_measurement')

        path = Path() 

        # Subscription to the required odom topic (edit accordingly)
        path = rospy.Subscriber('/path_odom', Path, callback)
        rate = rospy.Rate(30)  # 30hz

        try:
            while not rospy.is_shutdown():
                # rospy.spin()
                rate.sleep()
        except rospy.ROSInterruptException:
                pass
