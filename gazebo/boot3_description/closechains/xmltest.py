#!/usr/bin/env python

from __future__ import print_function
import math
import os
import numpy as np
import rospy

from lxml import etree

tree = etree.parse('/home/sietse/catkin_ws/src/Rowing/boot3_description/config/param.xacro-template')
root = tree.getroot()

etree.tostring(root)

spandict = root[0].attrib

print (spandict.get('name'))
print (spandict.get('value'))

tochange = ['span', 'hip2', 'elbow_left']
tovalue  = [ '1.0', '4', '56']

for jnt in tochange:
    value = tovalue[tochange.index(jnt)]
    for child in root:
        if child.attrib['name'] == jnt:
            child.attrib['value'] = value


for child in root:
    print(child.attrib)

#tree.write('new_param.xacro')

    


