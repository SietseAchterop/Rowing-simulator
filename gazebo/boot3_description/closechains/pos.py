#!/usr/bin/env python

""" 
set positions of boot3
"""

from __future__ import print_function
import math
import os
import numpy as np
import rospy

from sensor_msgs.msg import JointState


# All 19 joints to control
#   Let op de volgorde! Dit is hoe het nu in joint_states staat in mijn geval!
arm_joints = [
    'knee', 'hip1', 'hip2', 'back_lh', 'shoulder_l', 'shoulder_psi_l', 'shoulder_theta_l', 'elbow_theta_l', 'elbow_left', 'handle_left_j1', 'handle_left_j2',
    'handle_left_j3', 'shoulder_r', 'shoulder_psi_r', 'shoulder_theta_r', 'elbow_theta_r', 'elbow_right', 'handle_right_j1', 'handle_right_j2', 'handle_right_j3' ]
"""
voor inzet  0
na inzet    1
eind haal   2
na uitzet   3
"""
pos_joints = [
    [ 0,     0,      0,      0,         0,           0,                0,                  0,               0,            0,                 0,
      0,               0,            0,                 0,                 0,              0,             0,                   0,                 0 ],
    [ 0,     0,      0,      0,        -0.22,        0.25,            -0.07,              -0.36,            0.26,        -0.01,              0.21,
      0.12,           -0.07,            0.01,          -0.04,             -0.23,           0.05,          0.14,                0.21,              0 ],
    [ 0,     0,      0,      0,         0,           0,                0,                  0,               0,            0,                 0,
      0,               0,            0,                 0,                 0,              0,             0,                   0,                 0 ],
    [ 0,     0,      0,      0,         0,           0,                0,                  0,               0,            0,                 0,
      0,               0,            0,                 0,                 0,              0,             0,                   0,                 0 ]]


# Other joints
others = [      'prism', 'rev1', 'rev2', 'starboard_rowlock', 'port_rowlock', 'starboard_oar', 'port_oar', 'starboard_blade', 'port_blade', 'sliding', 'fp_ll' ]
pos_others = [ [ 0,       0,      0,      0,                   0,              0,               0,          0,                 0,            0,         0 ],
               [ 0,       0,      0,      0,                   0,             -0.11,            0,          0,                 0,            0,         0 ],
               [ 0,       0,      0,      0,                   0,              0,               0,          0,                 0,            0,         0 ],
               [ 0,       0,      0,      0,                   0,              0,               0,          0,                 0,            0,         0 ] ]

""" main program """

if __name__ == '__main__':
    rospy.init_node('set_positions')
    args = rospy.myargv()
    rate = rospy.Rate(10)
    
    n =int(args[1])

    num_joints = len(arm_joints)+len(others)
    pub = rospy.Publisher('joint_states', JointState, queue_size=5)

    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.position = num_joints * [0.0]
    msg.velocity = num_joints * [0.0]
    msg.effort = num_joints * [0.0]
    msg.name = others+arm_joints

    # select position
    msg.position = pos_others[n] + pos_joints[n]
    print (msg)

    while not rospy.is_shutdown():
      msg.header.stamp = rospy.Time.now()
      pub.publish(msg)
      rate.sleep()
    print("Done")

    
