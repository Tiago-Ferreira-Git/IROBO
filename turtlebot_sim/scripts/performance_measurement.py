#!/usr/bin/env python3
from __future__ import print_function
import rospy
import numpy as np
import scipy.io
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

import time


data_odom = np.zeros((1,2))
data_ekf = np.zeros((1,2))
data_gt = np.zeros((1,2))

def callback_odom(data):
        global data_odom
        for poses in data.poses:
              data_odom = np.append(data_odom,[[poses.pose.position.x, poses.pose.position.y]], axis=0)
        return data.poses


def callback_ekf(data):
        global data_ekf
        for poses in data.poses:
              data_ekf = np.append(data_ekf,[[poses.pose.position.x, poses.pose.position.y]], axis=0)
        return data.poses

def callback_gt(data):
        global data_gt
        for poses in data.poses:
              data_gt = np.append(data_gt,[[poses.pose.position.x, poses.pose.position.y]], axis=0)
        return data.poses


if __name__ == '__main__':
        
        rospy.init_node('performance_measurement')

        path = Path() 
        teste = PoseStamped()

        # Subscription to the required odom topic (edit accordingly)
        path = rospy.Subscriber('/path_odom', Path, callback_odom)
        path2 = rospy.Subscriber('/path_ekf', Path, callback_ekf)
        path3 = rospy.Subscriber('/ground_truth', Path, callback_gt)
        rate = rospy.Rate(30)  # 30hz

        try:
            while not rospy.is_shutdown():
                # rospy.spin()
                rate.sleep()
        except rospy.ROSInterruptException:
                scipy.io.savemat('test.mat', {'odom': data_odom,'gt': data_gt,'ekf': data_ekf})
                print(data_odom.shape)
                pass
