#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped
import numpy as np
import scipy.io
from nav_msgs.msg import Odometry, Path

data_odom = np.zeros((1,2))

data_gt = np.zeros((1,2))

def callback_odom(data):
        global data_odom
        data_odom = np.append(data_odom,[[data.poses[0].pose.position.x, data.poses[0].pose.position.y]], axis=0)
        return data.poses


def cb(event):
    global data_gt
    stamp = rospy.Time.now()
    try:
        listener.waitForTransform(src_frame, dst_frame, stamp,
                                timeout=rospy.Duration(1))
    except Exception as e:
        rospy.logerr(e)
        return

    dst_pose = listener.lookupTransform(src_frame, dst_frame, stamp)
    data_gt = np.append(data_gt,[[dst_pose[0][0], dst_pose[0][1]]], axis=0)
    


if __name__ == '__main__':
    rospy.init_node('tf_to_pose')
    src_frame = rospy.get_param('~src_frame')
    dst_frame = rospy.get_param('~dst_frame')
    #rate = rospy.get_param('~rate', 1.)
    listener = tf.TransformListener()
    path = rospy.Subscriber('/path_odom', Path, callback_odom)
    timer = rospy.Timer(rospy.Duration(1.0 / 1000), cb)
    rate = rospy.Rate(1000)  # 50hz

    try:
        while not rospy.is_shutdown():
                # rospy.spin()
                rate.sleep()
    except rospy.ROSInterruptException:
        scipy.io.savemat('plotting_data.mat', {'ground_truth': data_gt,'odom':data_odom})
        print(data_odom.shape)
        pass
