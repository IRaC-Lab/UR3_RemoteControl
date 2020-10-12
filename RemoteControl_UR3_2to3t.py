#! /usr/bin/env python3

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
import math


from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import *
from math import pi
from math import *

import serial
from numpy import * 

import matplotlib.pyplot as plt
from math import * 
import threading

from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Point, Pose
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, MyoPose, EmgArray

 
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
	       'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

start_position = [-1.57, -0.351, 0, -2.54818 , -1.57, 1.57]   

Q = [-1.57,-0.351, 0, -2.54818 , -1.57, 1.57]                # This value will change constantly. 
						    	     # Q[0] is base.  Q[1] is shoulder 
						   	     # Q[2] is elbow. Q[3] is list 1
							     # Q[4] is list 2.Q[5] is list 3 
						             # We use Q[0] and Q[1]. 
myo_data_x = 0
myo_data_y = 0


data_s = []
step = 0
time.sleep(1)

client = None

x, y, z = [] , [], []

moving_time = 0





def callback(data):
	global myo_data_x
	global myo_data_y
	rospy.loginfo("!!!Success Myo!!!")
	myo_data_x = data.x
	myo_data_y = data.y
	 



	time.sleep(1)

def callback_process(data):
	global x,y,z 
	global step 
	global data_s

	data_s = data.data

	w = 0

	rospy.loginfo(data_s)
	 


	print(data_s)


	time.sleep(0.1)
	step = step + 1
	""" f = open("test_gest.txt","a")
	f.write(data_s + "\n") 
	f.close """ # save data.





def listener():					# date received by sensor 


	rospy.Subscriber('/myo_raw/myo_ori', Vector3,callback, queue_size=1)
	rospy.Subscriber('/myo_raw/myo_gest_str', String, callback_process, queue_size=10)
	rospy.spin()


print("Start !!! \n")



def start_point(): 
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        g.trajectory.points = [
		JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
		JointTrajectoryPoint(positions=start_position, velocities=[0]*6, time_from_start=rospy.Duration(5.0))]



        client.send_goal(g)
        client.wait_for_result()
    
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise




def move_point():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        g.trajectory.points = [
		JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
		JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(0.01))]     # 0.01s is the minimum unit time to move the robot.



        client.send_goal(g)
        client.wait_for_result()
    
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def gesture_move():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        g.trajectory.points = [
		JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
		JointTrajectoryPoint(positions=Q, velocities=[0]*6, time_from_start=rospy.Duration(1.5))]



        client.send_goal(g)
        client.wait_for_result()
    
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise
    
   
def whilemove():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES

    data_box_x = []      # Myo armband's orientation data.
    data_box_y = []

    boxx = 0           # Total angle to move
    box_x = 0          # Angle to move
    boxy = 0   
    box_y = 0
    
    global Q
    global myo_data_x, myo_data_y, step
    global moving_time
    global data_s
    angle_ra_x = 0
    angle_ra_y = 0
    data_cnt=0              # Sensor data count

    base_joint = -1.57         
    shoulder_joint = -0.351 
    
    xx = 0
    xy = 0
    xx_box , xy_box = [], []

    yx = 0
    yy = 0
    yx_box , yy_box = [], []

    angle_sub_x = 0
    angle_sub_y = 0
    	
    time.sleep(3)
    print("You can move now !!! ")

    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position

        while True :	
	
                data_box_x.append(myo_data_x)
                data_box_y.append(myo_data_y)

                angle_ra_x = data_box_x[-1]
                angle_ra_y = data_box_y[-1]

                xx = cos(angle_ra_x)
                xy = sin(angle_ra_x)

                yx = cos(angle_ra_y)
                yy = sin(angle_ra_y)

                xx_box.append(xx)
                xy_box.append(xy)


                yx_box.append(yx)
                yy_box.append(yy)

                if(data_cnt > 2):
		
                        angle_sub_x = math.asin((xx_box[-2]*xy_box[-1]) - (xy_box[-2]*xx_box[-1]))   # X-axis -> UR3's base joint . 

                        if(angle_sub_x > 0.03):                                    ### This part is a UR3's velocity limit part.  
                                boxx = boxx + (angle_sub_x - 0.03) 		   ### *UR3's maximum velocity is about 0.0314radian/0.01s.
                                angle_sub_x = 0.03
	
                        if(angle_sub_x < -0.03):
                                boxx = boxx + (angle_sub_x + 0.03) 
                                angle_sub_x = -0.03
	
                        if(boxx != 0 ):
                                        if(boxx > 0 ):
                                                if(boxx > 0.02):
                                                        boxx = boxx - 0.02
                                                        box_x = 0.02
                                                else :
							
                                                        box_x = boxx
                                                        boxx = 0 
                                        elif(boxx < 0 ):
                                                if(boxx < -0.02):
                                                        boxx = boxx + 0.02
                                                        box_x = -0.02
                                                else:
                                                        box_x = boxx
                                                        boxx = 0
		
                        print("X : ",angle_sub_x)
                        print("box : ",boxx)
	

                        base_joint = base_joint - (angle_sub_x + box_x)
                        box_x = 0 

			
                        angle_sub_y = math.asin((yx_box[-2]*yy_box[-1]) - (yy_box[-2]*yx_box[-1]))  # Y-axis -> UR3's shoulder joint.
                        if(angle_sub_y > 0.03):
                                boxy = boxy + (angle_sub_y - 0.03) 
                                angle_sub_y = 0.03
	
                        if(angle_sub_y < -0.03):
                                boxy = boxy + (angle_sub_y + 0.03) 
                                angle_sub_y = -0.03
	
                        if(boxy != 0 ):
                                        if(boxy > 0 ):
                                                if(boxy > 0.02):
                                                        boxy = boxy - 0.02
                                                        box_y = 0.02
                                                else :
                                                        box_y = boxy
                                                        boxy = 0 
                                        elif(boxy < 0 ):
                                                if(boxy < -0.02):
                                                        boxy = boxy + 0.02
                                                        box_y = -0.02
                                                else:
                                                        box_y = boxy
                                                        boxy = 0
		
                        print("Y : ",angle_sub_y)
                        print("box : ",boxy)
                        shoulder_joint = shoulder_joint + (angle_sub_y + box_y)
                        box_y = 0
				

			

                Q[0] = base_joint		 # filter_data_box[-1] is the most recent data.
                Q[1] = shoulder_joint

                if data_s == "THUMB_TO_PINKY" :

                        '''Q[3] = - 3.0
                        gesture_move()

                        Q[3] = - 1.57
                        gesture_move()

                        Q[3] =  - 3.14
                        gesture_move()'''
	

                        time.sleep(1.0)

                print("base_joint : %s",base_joint)

                move_point()               # 

                data_cnt += 1
                step += 1

                client.send_goal(g)
                client.wait_for_result()
                time.sleep(0.01)


    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise




def main():
    global client
    th_ur = threading.Thread(target=whilemove)
    
    try:

        rospy.init_node("UR3_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print("Waiting for server...")
        client.wait_for_server()
        print("Connected to server")
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print("This program makes the robot move between the following three poses:")
        print(str([Q[i]*180./pi for i in range(0,6)]))

        print("Please make sure that your robot can move freely between these poses before proceeding!")

	    
        start_point()
	    

        th_ur.start()
        listener()



        print(str([Q[i]*180./pi for i in range(0,6)]))

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': 
        main()
