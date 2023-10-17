#!/usr/bin/env python3
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
import numpy as np
import sys
import json
from collections import deque
import scipy.io
import time
import tf
from tf2_ros import Buffer, TransformListener





data_gt = np.zeros((1,3))

data_odom = np.zeros((1,3))

data_ekf = np.zeros((1,3))

covariance = np.zeros((1,3))


erros = np.zeros((1,1))



class TransformHandler():

    def __init__(self, gt_frame, est_frame, max_time_between=0.01):
        self.gt_frame = gt_frame
        self.est_frame = est_frame
        self.frames = [gt_frame, est_frame]

        self.tf_buffer = Buffer(cache_time=rospy.Duration(max_time_between))
        self.__tf_listener = TransformListener(self.tf_buffer)

        #self.warn_timer = rospy.Timer(rospy.Duration(5), self.__warn_timer_cb)

    def __warn_timer_cb(self, evt):

        available_frames = self.tf_buffer.all_frames_as_string()
        avail = True
        for frame in self.frames:
            if frame not in available_frames:
                rospy.logwarn('Frame {} has not been seen yet'.format(frame))
                avail = False
        if avail:
            self.warn_timer.shutdown()

    def get_transform(self, fixed_frame, target_frame):
        # caller should handle the exceptions
        return self.tf_buffer.lookup_transform(target_frame, fixed_frame, rospy.Time(0))



def get_errors(transform):
    tr = transform.transform.translation
    return np.linalg.norm( [tr.x, tr.y] )


def callback(data):
        
        global data_odom
        data_odom = np.append(data_odom,[[data.header.stamp.to_sec(), data.pose.pose.position.x, data.pose.pose.position.y]], axis=0)
        return data
def callback_ekf(data):
        global covariance
        global data_ekf
        print(data.pose.covariance[1],)
        covariance = np.append(covariance,[[data.header.stamp.to_sec(), data.pose.covariance[0], data.pose.covariance[7]]], axis=0)
        data_ekf = np.append(data_ekf,[[data.header.stamp.to_sec(), data.pose.pose.position.x, data.pose.pose.position.y]], axis=0)
        return data

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
    data_gt = np.append(data_gt,[[stamp.to_sec(), dst_pose[0][0], dst_pose[0][1]]], axis=0)
    






if __name__ == '__main__':
        
        # Initializing node
        rospy.init_node('get_covariance')


        if rospy.rostime.is_wallclock():
            rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
            sys.exit(1)


        rospy.sleep(0.00001)
    
        src_frame = "odom"
        dst_frame = "mocap_laser_link"



        handler = TransformHandler(src_frame, dst_frame, max_time_between=20) # 500ms
        listener = tf.TransformListener()

        rate = rospy.Rate(1000)  # 50hz


        msg = Odometry()

        msg_ekf = Odometry()

        # Subscription to the required odom topic (edit accordingly)
        msg = rospy.Subscriber('/odom', Odometry, callback)

        msg_ekf = rospy.Subscriber('/odometry/filtered', Odometry, callback_ekf)


        timer = rospy.Timer(rospy.Duration(1.0 / 1000), cb)
        rate = rospy.Rate(1000)  # 30hz

        try:
                while not rospy.is_shutdown():
                    try:
                        #   gt_frame = "mocap_laser_link"
                        #   est_frame = "base_footprint"
                        t = handler.get_transform(src_frame, dst_frame)
                    except Exception as e:
                        rospy.logwarn(e)
                    else:
                        eucl = get_errors(t)
                        erros = np.append(erros,eucl)
                        #rospy.loginfo('Error (in mm): {:.2f}'.format(eucl * 1e3))

                    try:
                        rate.sleep()
                    except rospy.exceptions.ROSTimeMovedBackwardsException as e:
                        rospy.logwarn(e)
        except rospy.ROSInterruptException:
                scipy.io.savemat('plotting_data.mat', {'ekf': data_ekf,'odom':data_odom,'error':erros,'ground_truth':data_gt,'covariance':covariance})
                pass
    

    

