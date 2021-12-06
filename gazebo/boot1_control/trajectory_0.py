#!/usr/bin/env python

"""  trajectory voor boot1 

1 deze versie:
     parameterisen van de haal:tempo, haal/recover verhouding
   hoe recht laten gaan?
     sleuf, sturen, prismatic joint dus
   hoe verlopen de krachten tijdens de haal? wrijvingsparameters?


2 volgende:
   meerdere topics lezen en data verwerken
   eerst in array zetten 

   verwijderen en opnieuw laden model (met andere parameters!)
     ook programmatisch

 """

import math
import os
import numpy as np
import rospy
import actionlib
import matplotlib.pyplot as plt

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import WrenchStamped

# globals
# All joints to control
arm_joints = ['starboard_rowlock',
              'port_rowlock',
              'starboard_oar',
              'port_oar',
              'starboard_blade',
              'port_blade' ]
tfstart = 0

"""
 - topics to collect

    /gazebo/model_states
        position of the boat
    /boot1/joints_states
      position and effort of port_rowlock and starboard_rowlock
        to calculate strokes and energy input to the system
    /p_rl_sensor en sb_rl_sensor
        wat levert dit?

 - boatdata: groot genoeg voor de sessie maken

  eerste dim: aantal milliseconden
                moet integer zijn
              publish rate staat nu op 50, dus iedere 20 msec een callback
  tweede dim: aantal waarden:
      position and effort of starboard_rowlock, port_rowlock
       of effort uit sensor?
     position of boat

  
"""
mdata    = np.zeros((100,3))   # model data
bdata    = np.zeros((100,3))   # boat data
spdata   = np.zeros((100,3))   # sensor port data
ssbdata  = np.zeros((100,3))   # sensor starboard data

colldata = np.zeros((100,3))   # collected data

# counters for callbacks
mcnt   = 0
bcnt   = 0
spcnt  = 0
ssbcnt = 0
mstarttime   = 0
bstarttime   = 0
spstarttime  = 0
ssbstarttime = 0

stopcoll = False


# Connect to the trajectory action server
rospy.loginfo('Waiting for boot1 trajectory controller...')
arm_client = actionlib.SimpleActionClient('boot1/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

def jscallback(data):
    global bdata, bcnt, bstarttime, stopcoll
    jsts = data
    if stopcoll==False:
        timestamp = int(jsts.header.stamp.secs*1000 + jsts.header.stamp.nsecs/1000000)
        if bcnt ==0:
            bstarttime = timestamp
        if ((timestamp-bstarttime) % 20) != 0:
            # gebeurd vrijwel nooit
            print ('timestamp error', timestamp, timestamp-bstarttime)
        # port starboard rowlock
        p_port_rowlock = jsts.position[2]
        p_starboard_rowlock = jsts.position[5]
        e_port_rowlock = jsts.effort[2]
        e_starboard_rowlock = jsts.effort[5]
        #print ('position  %d   %1.2f %1.2f' % (timestamp, p_port_rowlock, p_starboard_rowlock))
        # data to nparray
        bdata[bcnt] = [timestamp-bstarttime, p_port_rowlock, p_starboard_rowlock, e_port_rowlock, e_starboard_rowlock]
        bcnt +=1

def modelcb(data):
    global mdata, mcnt, mstarttime, stopcoll
    md=data
    if stopcoll==False:
        #timestamp = int(md.header.stamp.secs*1000 + md.header.stamp.nsecs/1000000)
        timestamp = 0  # timestamp moet uit gazebo topic komen. later...
        if mcnt ==0:
            bstarttime = timestamp
        # no timestamps here
        position = md.pose[1].position.x
        mdata[mcnt] = [position, 0, 0]
        mcnt +=1


def spcb(data):
    global spdata, spcnt, spstarttime, stopcoll
    spd=data
    if stopcoll==False:
        timestamp = int(spd.header.stamp.secs*1000 + spd.header.stamp.nsecs/1000000)
        if spcnt ==0:
            spstarttime = timestamp
        if ((timestamp-spstarttime) % 20) != 0:
            # gebeurd vrijwel nooit
            print ('timestamp error', timestamp, timestamp-spstarttime)
        spdata[spcnt] = [timestamp-spstarttime, spd.wrench.force.x, spd.wrench.force.y, spd.wrench.force.z, spd.wrench.torque.x, spd.wrench.torque.y, spd.wrench.torque.z]
        spcnt +=1

        
def ssbcb(data):
    global ssbdata, ssbcnt, ssbstarttime, stopcoll
    ssbd=data
    if stopcoll==False:
        timestamp = int(ssbd.header.stamp.secs*1000 + ssbd.header.stamp.nsecs/1000000)
        if ssbcnt ==0:
            ssbstarttime = timestamp
        if ((timestamp-ssbstarttime) % 20) != 0:
            # gebeurd vrijwel nooit
            print ('timestamp error', timestamp, timestamp-ssbstarttime)
        ssbdata[ssbcnt] = [timestamp-ssbstarttime, ssbd.wrench.force.x, ssbd.wrench.force.y, ssbd.wrench.force.z, ssbd.wrench.torque.x, ssbd.wrench.torque.y, ssbd.wrench.torque.z]
        ssbcnt +=1



# main function
def main():
    global mdata, bdata, spdata, ssbdata, mcnt, bcnt, spcnt, ssbcnt, stopcoll
    rospy.init_node('trajectory_0')

    # restart (new) model from here
    # ook nog trajectory controller stoppen/starten: Ctrl-C
    #os.system("gz model -m boot1 -d; roslaunch boot1_control param.launch; sleep 1; rosrun gazebo_ros spawn_model -model boot1 -urdf -param robot_description")

    # Set to True to move back to the starting configurations
    reset  = rospy.get_param('~reset', False)
    repeat = rospy.get_param('~repeat', 3)

    # Set initial goal configurations for the boat
    move_goals  = [[0.0, -0.0, -0.0, 0.0, 0.0, -0.0]]
    time_goals  = [ 1.0]

    # describe a stroke cycle
    # use parameters?
    """
    cycle_goals = [[0.8, -0.8, -0.10, 0.10, 0.02, -0.01],
                   [0.8, -0.8, -0.28, 0.27 , 0.02, -0.01],
                   [-0.55, 0.55, -0.28, 0.27 , 0.02, -0.01],
                   [-0.55, 0.55, -0.10, 0.10 , 0.02, -0.01]]
    cycle_times = [ 0.2, 1.0, 0.2, 2.0 ]
    """
    cycle_goals = [[0.8, -0.8, -0.10, 0.10, 0.02, -0.01],
                   [-0.55, 0.55, -0.10, 0.10 , 0.02, -0.01]]
    cycle_times = [ 1.0, 2.0 ]

    # calculate complete session:
    for i in range(repeat):
        #print ('Pass %d' % i)
        for j in range(len(cycle_goals)):
            move_goals.append(cycle_goals[j])
            time_goals.append(cycle_times[j])

    # resize data
    tfstart = repeat * int(math.ceil(sum(cycle_times)))
    # currently 1 millisecond steps
    mdata   = np.resize(bdata, (tfstart*1000 + 400, 3))
    # currently 1/50-th second steps
    bdata   = np.resize(bdata, (tfstart*50 + 100, 5))
    spdata  = np.resize(bdata, (tfstart*50 + 100, 7))
    ssbdata = np.resize(bdata, (tfstart*50 + 100, 7))

    # subscribe to ...
    rospy.Subscriber("/boot1/joint_states", JointState, jscallback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, modelcb)
    rospy.Subscriber("/p_rl_sensor", WrenchStamped, spcb)
    rospy.Subscriber("/sb_rl_sensor", WrenchStamped, ssbcb)

    # connect to server
    arm_client.wait_for_server()
    #rospy.loginfo('...connected.')
        
    do_move(move_goals, time_goals)
    # sleep until movement completely done to collect the data
    print('start sleep')
    rospy.sleep(tfstart)
    print('stop sleep and collecting')
    
    # stop collecting and process the data
    stopcoll = True

    np.set_printoptions(threshold=np.nan, precision=2)

    # waarom werkt dit, terwijl het geen global is in main????
    print ('cnt\'s: ', mcnt, bcnt, spcnt, ssbcnt)
    print('starttimes', mstarttime, bstarttime, spstarttime, ssbstarttime)
    # now collect data with proper times

    tmp=bdata[:bcnt, 1:3]
    #print ('boat data:', tmp)
    plt.subplot(5,1,1)
    #plt.xlabel('Time')
    plt.ylabel('Position')
    plt.plot(tmp)

    tmp=bdata[:bcnt, 3:5]
    plt.subplot(5,1,2)
    #plt.xlabel('Time')
    plt.ylabel('Effort')
    plt.plot(tmp)

    tmp=mdata[:mcnt, 0]
    plt.subplot(5,1,3)
    #plt.xlabel('Time')
    plt.ylabel('Boat position')
    plt.plot(tmp)

    tmp=spdata[:spcnt, 1:7]
    plt.subplot(5,1,4)
    #plt.xlabel('Time')
    plt.ylabel('Port sensor')
    plt.plot(tmp)

    tmp=ssbdata[:ssbcnt, 1:7]
    plt.subplot(5,1,5)
    #plt.xlabel('Time')
    plt.ylabel('Starboard sensor')
    plt.plot(tmp)

    # nu nog sensor en positie boot erbij

    plt.show()

def do_move(move_goals, time_goals):
    global tfstart
    #print ('Do move')

    # Create a single-point arm trajectory with the arm_goal as the end-point
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = arm_joints

    tfstart = 0
    for j in range(len(move_goals)):
        arm_goal = move_goals[j]
        tfstart = tfstart+time_goals[j]
        #print (' move_goals[%d] %s' % (tfstart, arm_goal))
        
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[j].positions = arm_goal
        arm_trajectory.points[j].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[j].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[j].time_from_start = rospy.Duration(tfstart)
    
    # Send the trajectory to the arm action server
    #rospy.loginfo('Start moving the boat, will take %0.1f seconds.' % tfstart)
        
    # Create an empty trajectory goal
    arm_goal = FollowJointTrajectoryGoal()
        
    # Set the trajectory component to the goal trajectory created above
    arm_goal.trajectory = arm_trajectory
        
    # Specify zero tolerance for the execution time
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
    # Send the goal to the action server
    arm_client.send_goal(arm_goal)
    arm_client.wait_for_result()   # why does this return immediately?

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
