#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("world", "Rdummy_link_2")

print(ik_solver.base_link)
print(ik_solver.tip_link)
print("Joint names: ")
print(ik_solver.joint_names)
print("Link names: ")
print(ik_solver.link_names)
print("Limits")
print(ik_solver.get_joint_limits())
