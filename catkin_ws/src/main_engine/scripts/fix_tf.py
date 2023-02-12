#!/usr/bin/env python3
import roslib
import rospy
from tf import transformations as tfs
import tf
import numpy

if __name__ == '__main__':
    rospy.init_node('fix_tf_broadcaster')
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    sleep_rate = rospy.Rate(10)
    while not rospy.is_shutdown():
      try:
        (trans,rot) = listener.lookupTransform('simulated/odom', 'simulated/base_footprint', rospy.Time(0))
        transform = tfs.concatenate_matrices(tfs.translation_matrix(trans), tfs.quaternion_matrix(rot))
        (trans_diff,rot_diff) = listener.lookupTransform('base_footprint', 'internal_odom', rospy.Time(0))
        transform_diff = tfs.concatenate_matrices(tfs.translation_matrix(trans_diff), tfs.quaternion_matrix(rot_diff))
        transform = numpy.matmul(transform, transform_diff)
        br.sendTransform(tfs.translation_from_matrix(transform),
           tfs.quaternion_from_matrix(transform),
                        rospy.Time.now(),
                        "internal_odom","odom")
        br.sendTransform( (0,0,0), (0,0,0,1),
                        rospy.Time.now(),
                        "simulated/odom","odom")
        (trans,rot) = listener.lookupTransform('simulated/head_rgbd_sensor_link', 'simulated/head_rgbd_sensor_rgb_frame', rospy.Time(0))
        br.sendTransform(trans, rot,
                        rospy.Time.now(),
                        "head_rgbd_sensor_rgb_frame","head_rgbd_sensor_link")
        (trans,rot) = listener.lookupTransform('simulated/head_rgbd_sensor_link', 'simulated/head_rgbd_sensor_depth_frame', rospy.Time(0))
        br.sendTransform(trans, rot,
                        rospy.Time.now(),
                        "head_rgbd_sensor_depth_frame","head_rgbd_sensor_link")
        (trans,rot) = listener.lookupTransform('simulated/hand_camera_rgb_frame', 'simulated/hand_camera_frame', rospy.Time(0))
        br.sendTransform(trans, rot,
                        rospy.Time.now(),
                        "hand_camera_rgb_frame","hand_camera_frame")
        (trans,rot) = listener.lookupTransform('simulated/head_center_camera_rgb_frame', 'simulated/head_center_camera_frame', rospy.Time(0))
        br.sendTransform(trans, rot,
                        rospy.Time.now(),
                        "head_center_camera_rgb_frame","head_center_camera_frame")
        (trans,rot) = listener.lookupTransform('simulated/head_l_stereo_camera_rgb_frame', 'simulated/head_l_stereo_camera_link', rospy.Time(0))
        br.sendTransform(trans, rot,
                        rospy.Time.now(),
                        "head_l_stereo_camera_rgb_frame", "head_l_stereo_camera_link")
        (trans,rot) = listener.lookupTransform('simulated/head_r_stereo_camera_rgb_frame', 'simulated/head_r_stereo_camera_link', rospy.Time(0))
        br.sendTransform(trans, rot,
                        rospy.Time.now(),
                        "head_r_stereo_camera_rgb_frame", "head_r_stereo_camera_link")
        sleep_rate.sleep()

      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    rospy.spin()