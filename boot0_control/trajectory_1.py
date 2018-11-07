#!/usr/bin/env python

"""  trajectory voor boot0  """

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# globals
# Which joints define the arm?
arm_joints = ['base_body',
              'body_arm',
              'arm_blad' ]

# Connect to the arm trajectory action server
rospy.loginfo('Waiting for boot0 trajectory controller...')
arm_client = actionlib.SimpleActionClient('boot0/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

# main function
def main():
    rospy.init_node('trajectory_0')
        
    # Set to True to move back to the starting configurations
    reset  = rospy.get_param('~reset', False)
    repeat = rospy.get_param('~repeat', 1)
    if reset:
        # Set the arm back to the resting position
        arm_goals  = [[0.0, 0.0, 0]]
        repeat = 1
    else:
        # Set a goal configurations for the arm

        arm_goals  = [[-0.84, 0.35, -0.12],
                      [-0.16, -0.66, 0.2],
                      [-0.33, -0.53, -0.18],
                      [-0.86, 0.28, -0.10],
                      [-0.84, 0.29, -0.12],
        ]
        
    arm_client.wait_for_server()
        
    rospy.loginfo('...connected.')
        
    # go to each goal a repeat number of times
    #   duration komt later

    for i in range(repeat):
        print ('Pass %d' % i)
        do_move(arm_goals)
        if reset:
            return
    print ('hoe snel?')
    
    arm_goals = [[0.0, 0.0, 0]]
    #do_move(arm_goals)
                 
    print ('Done')

def do_move(arm_goals):
    print ('Do move', arm_goals)

    # Create a single-point arm trajectory with the arm_goal as the end-point
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = arm_joints

    for j in range(len(arm_goals)):
        arm_goal = arm_goals[j]
        print (' arm_goals[%d] %s' % (j, arm_goal))
        
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[j].positions = arm_goal
        arm_trajectory.points[j].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[j].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[j].time_from_start = rospy.Duration(1+(j*1))
    
    # Send the trajectory to the arm action server
    rospy.loginfo('Moving the arm to next goal position...')
        
    # Create an empty trajectory goal
    arm_goal = FollowJointTrajectoryGoal()
        
    # Set the trajectory component to the goal trajectory created above
    arm_goal.trajectory = arm_trajectory
        
    # Specify zero tolerance for the execution time
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
    print('send')
    # Send the goal to the action server
    arm_client.send_goal(arm_goal)
    print('sended')
    arm_client.wait_for_result()
    print('klaar')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
