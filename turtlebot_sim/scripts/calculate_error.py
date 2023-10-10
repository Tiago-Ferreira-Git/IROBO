#!/usr/bin/env python3

import rospy
import rosbag
import sys
import argparse
import numpy
from tf2_ros import Buffer, TransformListener
from rosgraph_msgs.msg import Clock
import scipy.io

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



data_tf = numpy.zeros((1,2))
erros = numpy.zeros((1,1))

def get_errors(transform):
    global data_tf
    tr = transform.transform.translation
    data_tf = numpy.append(data_tf,[[tr.x, tr.y]],axis=0)
    return numpy.linalg.norm( [tr.x, tr.y] )




if __name__ == '__main__':

    rospy.init_node('evaluation_node')


    gt_frame = "mocap_laser_link"
    est_frame = "base_footprint"

    print(gt_frame)


    if rospy.rostime.is_wallclock():
        rospy.logfatal('You should be using simulated time: rosparam set use_sim_time true')
        sys.exit(1)

    rospy.loginfo('Waiting for clock')
    rospy.sleep(0.00001)

    handler = TransformHandler(gt_frame, est_frame, max_time_between=20) # 500ms

    rospy.loginfo('Listening to frames and computing error, press Ctrl-C to stop')
    sleeper = rospy.Rate(1000)
    try:
        while not rospy.is_shutdown():
            try:
                t = handler.get_transform(gt_frame, est_frame)
            except Exception as e:
                rospy.logwarn(e)
            else:
                eucl = get_errors(t)
                erros = numpy.append(erros,eucl)
                rospy.loginfo('Error (in mm): {:.2f}'.format(eucl * 1e3))

            try:
                sleeper.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException as e:
                rospy.logwarn(e)

    except rospy.exceptions.ROSInterruptException:

        scipy.io.savemat('tf.mat', {'tf_data': data_tf,'error':erros})
        pass
