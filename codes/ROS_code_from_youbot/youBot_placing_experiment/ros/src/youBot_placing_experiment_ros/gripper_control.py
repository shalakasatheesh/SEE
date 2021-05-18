#!/usr/bin/env python
"""
This module contains a component that ...TODO

"""
#-*- encoding: utf-8 -*-
__authors__ = 'Djordje Vukcevic, Aleksandar Mitrevski'

import rospy
import copy
import roslib
import std_msgs.msg
import geometry_msgs.msg
import brics_actuator.msg
from brics_actuator.msg import JointPositions, JointValue
     
def make_old_gripper_msg(state):

    # create joint positions message
    jp = JointPositions()
    
    # create joint values message for both left and right fingers
    jvl = JointValue()
    jvr = JointValue()
    
    # Fill in the gripper positions desired
    jvl.joint_uri = "gripper_finger_joint_l"
    jvl.unit = 'm'
    jvl.value = state[0]
    jvr.joint_uri = "gripper_finger_joint_r"
    jvr.unit = 'm'
    jvr.value = state[1]
    
    # Append those onto JointPositions
    jp.positions.append(copy.deepcopy(jvl))
    jp.positions.append(copy.deepcopy(jvr))
    
    return jp

def open_old_gripper(gripper_publisher, gripper_msg_rate):
    time = rospy.get_rostime()
    t0 = time.secs
    rospy.loginfo('Trying to open gripper...')

    while not rospy.is_shutdown():
        rospy.loginfo("moving gripper")
        gripper_publisher.publish(make_old_gripper_msg([0.0115, 0.0115]))
        gripper_msg_rate.sleep()
        time = rospy.get_rostime() 
        t1 = time.secs
        if (t1-t0) > 1:
            break

def close_old_gripper(gripper_publisher, gripper_msg_rate):

    time = rospy.get_rostime()
    t0 = time.secs
    rospy.loginfo('Trying to close gripper')

    while not rospy.is_shutdown():
        rospy.loginfo("moving gripper")
        gripper_publisher.publish(make_old_gripper_msg([0.01, 0.001]))
        gripper_msg_rate.sleep()
        time = rospy.get_rostime() 
        t1 = time.secs
        if (t1-t0) > 1:
            break

def talker():
    rospy.init_node('gripper_controller', anonymous=True)

    gripper_publisher = rospy.Publisher("/arm_1/gripper_controller/position_command", JointPositions, queue_size = 10)

    # setup the loop rate for controlling gripper
    gripper_msg_rate = rospy.Rate(1) # 10hz

 
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        motion = raw_input('Chose desired type of gripper motion: 1 for close, 2 for open, or exit the program with 3: ') 

        if motion.isdigit():
            print('You entered: '+ str(motion))
            if int(motion) == 1:
                close_old_gripper(gripper_publisher, gripper_msg_rate)
            elif int(motion) == 2:
                open_old_gripper(gripper_publisher, gripper_msg_rate)
            elif int(motion) == 3:
                print('Exiting the node!')
                exit()
            else:
                print('Not a valid option, you must choose between 3 valid options: 1, 2, 3')
                exit() 

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
