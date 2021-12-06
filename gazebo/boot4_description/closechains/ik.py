#!/usr/bin/env python

"""
Gebruiken met
   roslaunch boot4_description boot_rviz.launch reconnectloops:=true

get joint states using joint_state_publisher
get tf between 2 links
calc trac_ik
write new settings to param file

deze code 2 keer uitvoeren, met Ldummies en Rdummies

"""

from __future__ import print_function
import math
import os
import numpy as np
import rospy
import tf
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from lxml import etree

# start program
rospy.init_node('calc_new_positions', anonymous=True)

# collect some joint state data
js = JointState()
cbcount = 0
def callbackjs(data):
    global cbcount, js
    if cbcount < 2:
        js = data
    cbcount +=1
rospy.Subscriber("joint_states", JointState, callbackjs)

# wait until js is being filled
while cbcount < 2:
    pass
number_of_joints = len(js.name)
# we now have the needed joint state data

# origin of chain
fromlink = "seat"
# goal (connect the arm to the oar)
tolink   = "Rdummy_link_1"
# for this link
tolink2   = "Rdummy_link_2"
# both dummy_link should be on the same spot, then the shoulder is connected properly

# calculate needed position
listener = tf.TransformListener()
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform(fromlink, tolink, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    
    print("Transformation")
    print(trans)
    print(rot)
    break
# we now have the needed transformation

# try to place tolink2 on position tolink
ik_solver = IK(fromlink, tolink2, epsilon=1e-07)
seed_state = [0.0] * ik_solver.number_of_joints
res = ik_solver.get_ik(seed_state,
                       trans[0], trans[1], trans[2],
                       rot[0], rot[1], rot[2], rot[3] )

# if succeeded...
if res != None:
    print("Link names: ")
    print(ik_solver.link_names)
    print()
    print("Joint names: ")
    print(ik_solver.joint_names)
    print()
    print(res)

    # get template file
    tree = etree.parse('/home/sietse/catkin_ws/src/Rowing/boot4_description/config/param.xacro-template')
    root = tree.getroot()

    # note: tf and ik_solver use current joint states, and we calculate with param-template values!
    #       so this only works we we don't change any joints ourselves.
    for jnt in ik_solver.joint_names:
        for child in root:
            if type(child.attrib) is etree._Attrib:
                if child.attrib['name'] == jnt:
                    value = float(child.attrib['value']) + float(res[ik_solver.joint_names.index(jnt)])
                    print (jnt, 'changed from', child.attrib['value'], ' to ', "{:.4f}".format(value))
                    child.attrib['value'] = "{:.4f}".format(value)

    #for child in root:
    #    print(child.attrib)

    # set new parameters
    tree.write('/home/sietse/catkin_ws/src/Rowing/boot4_description/config/param.xacro')

else:
    print("No result from trac_ik, nothing changed.")

