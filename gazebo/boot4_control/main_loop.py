#!/usr/bin/env python

"""  main loop for experiments

for c in args[2:]:
            controller_manager_interface.load_controller(c)
        for c in args[2:]:
            controller_manager_interface.start_controller(c)

 """

from __future__ import print_function
import math
import os
import numpy as np
import rospy
import actionlib
import matplotlib.pyplot as plt

try:
    import readline
except:
    pass #readline not available

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import WrenchStamped
from controller_manager import controller_manager_interface

help_string = """   Commands:
     <empty>,
     g, r, h,
     p, q, list, (un)load, stop, start, model
"""


# globals
# All 17 joints to control (ook de handle joints?)
arm_joints = [
    'knee', 'hip2', 'back_lh', 'starboard_blade', 'port_blade',
    'shoulder_l', 'shoulder_psi_l', 'shoulder_theta_l', 'elbow_xx_l', 'elbow_theta_l', 'elbow_left',
    'shoulder_r', 'shoulder_psi_r', 'shoulder_theta_r', 'elbow_xx_r', 'elbow_theta_r', 'elbow_right' ]
tfstart = 0

"""
 - topics to collect

    /gazebo/model_states
        position of the boat
    /boot/joints_states
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
bdata    = np.zeros((100,3))   # boat data (joint states)
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

coll_running = False
contr_running = False

def jscallback(data):
    global bdata, bcnt, bstarttime, coll_running
    jsts = data
    if coll_running:
        timestamp = int(jsts.header.stamp.secs*1000 + jsts.header.stamp.nsecs/1000000)
        if bcnt ==0:
            bstarttime = timestamp
        #print ('jscallback times', timestamp-bstarttime)
        # collect needed data. directly use indices for efficiency
        p_knee = jsts.position[15]
        hip2 = jsts.position[14]
        e_port_rowlock = 0#jsts.effort[2]
        e_starboard_rowlock = 0#jsts.effort[5]
        #print ('position  %d   %1.2f %1.2f' % (timestamp, p_knee, hip2))
        # data to nparray
        bdata[bcnt] = [timestamp-bstarttime, p_knee, hip2, e_port_rowlock, e_starboard_rowlock]
        bcnt +=1

def modelcb(data):
    global mdata, mcnt, mstarttime, coll_running
    md=data
    if coll_running:
        #timestamp = int(md.header.stamp.secs*1000 + md.header.stamp.nsecs/1000000)
        timestamp = 0  # timestamp moet uit gazebo topic komen. later...
        if mcnt ==0:
            bstarttime = timestamp
        # no timestamps here
        position = md.pose[1].position.x
        mdata[mcnt] = [position, 0, 0]
        mcnt +=1

def spcb(data):
    global spdata, spcnt, spstarttime, coll_running
    spd=data
    if coll_running:
        timestamp = int(spd.header.stamp.secs*1000 + spd.header.stamp.nsecs/1000000)
        if spcnt ==0:
            spstarttime = timestamp
        #print ('spcb times', timestamp-spstarttime)
        spdata[spcnt] = [timestamp-spstarttime, spd.wrench.force.x, spd.wrench.force.y, spd.wrench.force.z, spd.wrench.torque.x, spd.wrench.torque.y, spd.wrench.torque.z]
        spcnt +=1

        
def ssbcb(data):
    global ssbdata, ssbcnt, ssbstarttime, coll_running
    ssbd=data
    if coll_running:
        timestamp = int(ssbd.header.stamp.secs*1000 + ssbd.header.stamp.nsecs/1000000)
        if ssbcnt ==0:
            ssbstarttime = timestamp
        #print ('ssbcb times', timestamp-ssbstarttime)
        ssbdata[ssbcnt] = [timestamp-ssbstarttime, ssbd.wrench.force.x, ssbd.wrench.force.y, ssbd.wrench.force.z, ssbd.wrench.torque.x, ssbd.wrench.torque.y, ssbd.wrench.torque.z]
        ssbcnt +=1



def create_movegoals():
    global repeat
    # Initially go to proper position (use arm_joints order)
    move_goals  = [[0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    time_goals  = [ 1.0]

    # default values
    cycle_goals =  [ [0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [-2.46, 0.34, 0.30, 0.0, 0.0,
                      1.85, 0.29, 0.00, -1.77, -0.16, 0.0,
                       1.84, 0.30, 0.0, -1.67, -0.16, -0.07] ]
    cycle_times = [ 1.0, 1.0 ]

    '''
                      [0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    '''

    # calculate complete session:
    for i in range(repeat):
        #print ('Pass %d' % i)
        for j in range(len(cycle_goals)):
            move_goals.append(cycle_goals[j])
            time_goals.append(cycle_times[j])
    return (move_goals, time_goals)    

def do_move(move_goals, time_goals):
    #print ('Do move')

    # Create a single-point arm trajectory with the arm_goal as the end-point
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = arm_joints

    time_goal = 0
    for j in range(len(move_goals)):
        arm_goal = move_goals[j]
        time_goal += time_goals[j]
        print (' arm_goal, time_goal', arm_goal, time_goal)
        
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[j].positions = arm_goal
        arm_trajectory.points[j].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[j].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[j].time_from_start = rospy.Duration(time_goal)
    
    # Create an empty trajectory goal
    arm_goal = FollowJointTrajectoryGoal()
        
    # Set the trajectory component to the goal trajectory created above
    arm_goal.trajectory = arm_trajectory
        
    # Specify zero tolerance for the execution time
    arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
    # Send the goal to the action server
    arm_client.send_goal(arm_goal)
    print('send goal')
    arm_client.wait_for_result()   # why does this return immediately?
    print('wait done')

def start_experiment():
    global coll_running, tfstart, mdata, bdata, spdata, ssbdata, mcnt, bcnt, spcnt, ssbcnt
    print('start experiment')

    move_goals, time_goals = create_movegoals()

    # resize data
    tfstart = int(math.ceil(sum(time_goals)))
    rospy.loginfo('Start moving the boat, will take %0.1f seconds.' % tfstart)
    # currently 1 millisecond steps
    mdata   = np.resize(bdata, (tfstart*4000 + 12000, 3))
    # currently 1/50-th second steps
    bdata   = np.resize(bdata, (tfstart*50 + 2000, 5))
    spdata  = np.resize(bdata, (tfstart*50 + 2000, 7))
    ssbdata = np.resize(bdata, (tfstart*50 + 2000, 7))

    print ('start', mcnt,bcnt,spcnt, ssbcnt)
    coll_running = True

    do_move(move_goals, time_goals)
    # sleep until movement completely done to collect the data
    print('start sleep')
    rospy.sleep(tfstart)
    print('stop sleep and collecting')
    
    # stop collecting and process the data
    coll_running = False

    #    np.set_printoptions(threshold=np.nan, precision=2)

    # waarom werkt dit, terwijl het geen global is in main????
    print ('cnt\'s: ', mcnt, bcnt, spcnt, ssbcnt)
    print('starttimes', mstarttime, bstarttime, spstarttime, ssbstarttime)
    # now collect data with proper times

    tmp=bdata[:bcnt, 1:3]
    #print ('boat data:', tmp)
    plt.subplot(5,1,1)
    #plt.xlabel('Time')
    plt.ylabel('Knee, hip2')
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


def changeparam():
    global repeat
    a=raw_input('Give enter after changing parameters.')
    # now only repeat
    repeat = int(a)

def list_controllers():
    os.system("rosservice call /boot/controller_manager/list_controllers")

def load_controllers():
    os.system("rosservice call /boot/controller_manager/load_controller joint_state_controller")
    os.system("rosservice call /boot/controller_manager/load_controller joint_trajectory_controller")
    # wachten tot de topics weer lopen lijkt nodig.... of buffer groter maken

def stop_controllers():
    os.system("rosservice call /boot/controller_manager/switch_controller \"{stop_controllers: ['joint_state_controller', 'joint_trajectory_controller'], start_controllers: [], strictness: 2}\"")

def start_controllers():
    os.system("rosservice call /boot/controller_manager/switch_controller \"{start_controllers: ['joint_state_controller', 'joint_trajectory_controller'], stop_controllers: [], strictness: 2}\" ")

def unload_controllers():
    os.system("rosservice call /boot/controller_manager/unload_controller joint_state_controller")
    os.system("rosservice call /boot/controller_manager/unload_controller joint_trajectory_controller")


def reload_model():
    os.system('roslaunch boot4_control param.launch')
    os.system('rosparam load ~/catkin_ws/src/Rowing/boot4_control/config/boot_trajectory.yaml')
    os.system("gz model -m boot -d; sleep 1; rosrun gazebo_ros spawn_model -model boot -urdf -param robot_description")

def world_pause(state):
    print("pause: ", state)
    if state:
        os.system("gz world -p 1")
    else:
        os.system("gz world -p 0")

""" main program """

if __name__ == '__main__':
    rospy.init_node('main_loop')
    args = rospy.myargv()

    plt.style.use('ggplot') # nicer plots?
    
    # Set to True to move back to the starting configurations
    reset  = rospy.get_param('~reset', False)
    repeat = rospy.get_param('~repeat', 1)

    print("Make sure the model is started with 'roslaunch boot_gazebo control_world.launch' ")
    world_pause(False)
    #    we could start gazebo from here...
    #load_controllers()

    # Connect to the trajectory action server
    #rospy.loginfo('Waiting for boot trajectory controller...')
    arm_client = actionlib.SimpleActionClient('/boot/joint_trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    # subscribe to ...
    rospy.Subscriber("/boot/joint_states", JointState, jscallback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, modelcb)
    rospy.Subscriber("/p_rl_sensor", WrenchStamped, spcb)
    rospy.Subscriber("/sb_rl_sensor", WrenchStamped, ssbcb)

    # connect to server
    arm_client.wait_for_server()
    #rospy.loginfo('...connected.')

    # command loop
    print (help_string)
    while not rospy.is_shutdown():
        next = raw_input('-> ')
        if next == '' or next.split()[0] == 'g':
            mcnt=0; bcnt=0; spcnt=0; ssbcnt=0
            start_experiment()
        elif next.split()[0] == 'p':
            changeparam()
        elif next.split()[0] == 'r':
            stop_controllers()
            unload_controllers()
            world_pause(True)
            reload_model()
            load_controllers()
            world_pause(False)
        elif next.split()[0] == 'q':
            break
        elif next.split()[0] == 'h':
            print (help_string)
        elif next.split()[0] == 'list':
            list_controllers()
        elif next.split()[0] == 'load':
            load_start_controllers()
        elif next.split()[0] == 'unload':
            unload_controllers()
        elif next.split()[0] == 'stop':
            stop_controllers()
        elif next.split()[0] == 'start':
            start_controllers()
        elif next.split()[0] == 'model':
            reload_model()
        else:
            print('command error')
            
    print('Command loop ended.')
    
    # cleanup


    
