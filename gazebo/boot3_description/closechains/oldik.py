#!/usr/bin/env python

from trac_ik_python.trac_ik import IK

ik_solver = IK("skiff",
                              "Ldummy_link_1", epsilon=1e-02)

seed_state = [0.0] * ik_solver.number_of_joints

"""   dummy link 1
print(ik_solver.get_ik(seed_state,
                                 -3.591, 0.191, 0.547, 
                                 0.347, 0.935, 0.066, 0.042))
   dummy link 2
"""
print(ik_solver.get_ik(seed_state,
                                 -3.577, 0.174, 0.526, 
                                 0.358, 0.933, 0.033, 0.037))
