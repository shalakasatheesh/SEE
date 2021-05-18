#!/usr/bin/env python
"""
This module contains a component that ...TODO

"""
#-*- encoding: utf-8 -*-
__authors__ = 'Djordje Vukcevic'

import rospy
import copy
import roslib
import std_msgs.msg
import moveit_msgs.msg
import moveit_commander
import geometry_msgs.msg
import brics_actuator.msg
from brics_actuator.msg import JointPositions, JointValue

class youBotPlacing(object):

    def __init__(self):

        #Start and end state of the robot
        self.start_state = rospy.get_param('~start_state', None)
        self.yb_number = rospy.get_param('~yb_number', None)

        # MoveIt! interface
        arm_name = rospy.get_param('~arm', None)
        assert arm_name is not None, "The group to be moved must be specified (e.g. arm_1)."

        # Set up MoveIt!
        self.arm = moveit_commander.MoveGroupCommander(arm_name)

        self.gripper_publisher = rospy.Publisher("/arm_1/gripper_controller/position_command", JointPositions, queue_size = 10)
	self.rosbag_recorder_publisher = rospy.Publisher('/mcr_tools/rosbag_recorder/event_in', std_msgs.msg.String, queue_size = 1)

        # setup the loop rate for controlling gripper
        self.gripper_msg_rate = rospy.Rate(1) # 10hz
     
    def make_old_gripper_msg(self, state):

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

    def start(self):
        """
        Starts the component.
        """

        self.arm.clear_pose_targets()

        self.arm.set_start_state_to_current_state()

        #1) Set target configuration with the name of robot start state.
        # Making sure that robot starts from folded configuration
        try:
            self.arm.set_named_target(self.start_state)
        except Exception as e:
            rospy.logerr('unable to set start configuration: %s' % (str(e)))
            return False

        rospy.loginfo('Planning motion and trying to move arm in folded configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for folded configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for folded configuration')
            return True

    

    def open_old_gripper(self):
        
        time = rospy.get_rostime()
        t0 = time.secs
        rospy.loginfo('Trying to open gripper...')

        while not rospy.is_shutdown():
            rospy.loginfo("moving gripper")

            if self.yb_number == 3:
                self.gripper_publisher.publish(self.make_old_gripper_msg([0.0115, 0.0115]))

            elif self.yb_number == 1:
                self.gripper_publisher.publish(self.make_old_gripper_msg([0.0115, 0.0115]))

            self.gripper_msg_rate.sleep()
            time = rospy.get_rostime() 
            t1 = time.secs
            if (t1-t0) > 1:
                break

    
    def close_old_gripper(self):
        
        time = rospy.get_rostime()
        t0 = time.secs
        rospy.loginfo('Trying to close gripper')

        while not rospy.is_shutdown():
            rospy.loginfo("moving gripper")
            self.gripper_publisher.publish(self.make_old_gripper_msg([0.003, 0.003]))
            self.gripper_msg_rate.sleep()
            time = rospy.get_rostime() 
            t1 = time.secs
            if (t1-t0) > 1:
                break

    def go_pregrasp(self):

        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for pregrasp pose
        if self.yb_number == 3:
            group_variable_values[0] = 1.40319
            group_variable_values[1] = 2.29624
            group_variable_values[2] = -0.36571
            group_variable_values[3] = 0.01424
            group_variable_values[4] = 2.92550

        elif self.yb_number == 1:
            group_variable_values[0] = 4.55037
            group_variable_values[1] = 2.23517
            group_variable_values[2] = -0.44331
            group_variable_values[3] = 0.15688
            group_variable_values[4] = 2.92219
        
        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in pregrasp configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for pregrasp configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for pregrasp configuration')
            return True

    def go_postgrasp(self):

        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for postgrasp pose
        if self.yb_number == 3:
            group_variable_values[0] = 1.39322
            group_variable_values[1] = 1.92866
            group_variable_values[2] = -0.28814
            group_variable_values[3] = 0.31296
            group_variable_values[4] = 2.92564

        elif self.yb_number == 1:
            group_variable_values[0] = 4.54672
            group_variable_values[1] = 2.01843
            group_variable_values[2] = -0.40319
            group_variable_values[3] = 0.33356
            group_variable_values[4] = 2.92232

        
        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in postgrasp configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for postgrasp configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for postgrasp configuration')
            return True

    def place_straight(self):
        
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for straight pose
        if self.yb_number == 3:
            group_variable_values[0] = 1.39880
            group_variable_values[1] = 2.49
            group_variable_values[2] = -1.55904
            group_variable_values[3] = 1.04305
            group_variable_values[4] = 2.93579
        
        elif self.yb_number == 1:
            group_variable_values[0] = 4.56509
            group_variable_values[1] = 2.57875
            group_variable_values[2] = -1.90889
            group_variable_values[3] = 1.30451
            group_variable_values[4] = 2.93630

        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in straight configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for straight configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for straight configuration')
            return True

    def go_post_straight(self):
        
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for post_straight pose
        if self.yb_number == 3:
            group_variable_values[0] = 1.40218
            group_variable_values[1] = 2.33787
            group_variable_values[2] = -0.99027
            group_variable_values[3] = 0.61019
            group_variable_values[4] = 2.93957
        
        elif self.yb_number == 1:
            group_variable_values[0] = 4.56429
            group_variable_values[1] = 2.31808
            group_variable_values[2] = -1.23890
            group_variable_values[3] = 0.89515
            group_variable_values[4] = 2.93933

        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in post_straight configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for post_straight configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for post_straight configuration')
            return True

    def place_diagonal_left(self):
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for diagonal left pose
        if self.yb_number == 3:
            group_variable_values[0] = 0.97302
            group_variable_values[1] = 2.41
            group_variable_values[2] = -1.33293
            group_variable_values[3] = 0.88424
            group_variable_values[4] = 2.92190

        elif self.yb_number == 1:
            group_variable_values[0] = 4.19960
            group_variable_values[1] = 2.29393
            group_variable_values[2] = -1.0510
            group_variable_values[3] = 0.68004
            group_variable_values[4] = 2.93046
            #group_variable_values[0] = 3.94763
            #group_variable_values[1] = 2.35091
            #group_variable_values[2] = -1.31808
            #group_variable_values[3] = 0.93763
            #group_variable_values[4] = 2.9352
        
        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in diagonal left configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for diagonal left configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for diagonal left configuration')
            return True

    def go_post_diagonal_left(self):
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for post_diagonal_left pose
        if self.yb_number == 3:
            group_variable_values[0] = 0.97035
            group_variable_values[1] = 2.36871
            group_variable_values[2] = -0.87988
            group_variable_values[3] = 0.46066
            group_variable_values[4] = 2.92234

        elif self.yb_number == 1:
            group_variable_values[0] =  4.19960
            group_variable_values[1] =  2.29393
            group_variable_values[2] = -0.70000
            group_variable_values[3] =  0.40000
            group_variable_values[4] =  2.93046
            
        
        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in post_diagonal_left configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for post_diagonal_left configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for post_diagonal_left configuration')
            return True

    def place_diagonal_right(self):

        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for diagonal right pose
        if self.yb_number == 3:
            group_variable_values[0] = 1.92034
            group_variable_values[1] = 2.26
            group_variable_values[2] = -0.76276
            group_variable_values[3] = 0.46424
            group_variable_values[4] = 2.92241

        elif self.yb_number == 1:
            group_variable_values[0] = 4.94944
            group_variable_values[1] = 2.39084
            group_variable_values[2] = -1.46283
            group_variable_values[3] = 1.02411
            group_variable_values[4] = 2.91325
        
        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in diagonal right configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for diagonal right configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for diagonal right configuration')
            return True

    def go_post_diagonal_right(self):

        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()

        #Declaring variable for setting joint configuration
        group_variable_values = self.arm.get_current_joint_values()

        #Setting joint values for diagonal right pose
        if self.yb_number == 3:
            group_variable_values[0] = 1.92034
            group_variable_values[1] = 2.26873
            group_variable_values[2] = -0.41976
            group_variable_values[3] = 0.09559
            group_variable_values[4] = 2.92232

        elif self.yb_number == 1:
            group_variable_values[0] = 4.94921
            group_variable_values[1] = 2.30199
            group_variable_values[2] = -1.03793
            group_variable_values[3] = 0.68794
            group_variable_values[4] = 2.91413
        
        self.arm.set_joint_value_target(group_variable_values)

        rospy.loginfo('Planning motion and trying to move arm in post_diagonal_right configuration')
        success = self.arm.go(wait=True)
        if not success:
            rospy.logerr('Arm motion unsuccessful for post_diagonal_right configuration')
            return False
        else:
            rospy.loginfo('Arm motion successful for post_diagonal_right configuration')
            return True

    def start_recording(self, motion_id):
        pubish_str = 'e_start %d' % motion_id
        self.rosbag_recorder_publisher.publish(pubish_str)
    def stop_recording(self):
        pubish_str = 'e_stop'
        self.rosbag_recorder_publisher.publish(pubish_str)
    
def main():

    rospy.init_node('youBot_placing', anonymous=True)
    youBot_placing = youBotPlacing()

    youBot_placing.open_old_gripper()
    youBot_placing.start()

    while not rospy.is_shutdown():
        try:
            youBot_placing.go_pregrasp()

            while not rospy.is_shutdown():
                grasp = raw_input('\nGrasp now? Answer with YES to grasp the object and continue the experiment, or answer with NO to stop the experiment: ') 
                rospy.loginfo('\nYou entered: '+ str(grasp))
                if (grasp) == 'yes' or (grasp) == 'YES' or (grasp) == 'Yes' or (grasp) == 'Y' or (grasp) == 'y':
                    youBot_placing.close_old_gripper()
                    break 
                elif (grasp) == 'no' or (grasp) == 'NO' or (grasp) == 'No' or (grasp) == 'N' or (grasp) == 'n':
                    youBot_placing.start()
                    exit()
                else:
                    rospy.logerr('\nNot a valid option, you must choose between YES or NO!')
                    continue

            youBot_placing.go_postgrasp()

            while not rospy.is_shutdown():

                motion = raw_input('\nChose desired type of arm motion: 1 for Straight, 2 for Diagonal-Left and 3 for Diagonal-Right, or exit the program with 4: ') 
                
                if motion.isdigit():
                    rospy.loginfo('\nYou entered: '+ str(motion))

                    if int(motion) == 1:
			youBot_placing.start_recording(int(motion))
                        rospy.sleep(3)
                        youBot_placing.place_straight()
                        rospy.sleep(2)
                        youBot_placing.open_old_gripper()
                        rospy.sleep(5)
			youBot_placing.stop_recording()
                        #youBot_placing.go_post_straight()

                    elif int(motion) == 2:
			youBot_placing.start_recording(int(motion))
                        rospy.sleep(3)
                        youBot_placing.place_diagonal_left()
                        rospy.sleep(2)
                        youBot_placing.open_old_gripper()
                        rospy.sleep(5)
			youBot_placing.stop_recording()
                        #youBot_placing.go_post_diagonal_left()

                    elif int(motion) == 3:
			youBot_placing.start_recording(int(motion))
                        rospy.sleep(3)
                        youBot_placing.place_diagonal_right()
                        rospy.sleep(2)
                        youBot_placing.open_old_gripper()
                        rospy.sleep(5)
			youBot_placing.stop_recording()
                        #youBot_placing.go_post_diagonal_right()

                    elif int(motion) == 4:
                        youBot_placing.start()
                        exit()

                    else:
                        rospy.logerr('\nNot a valid option, you must choose between 3 valid options: 1, 2, 3, 4')
                        continue
                    break 

                else:
                    rospy.logerr('\nNot a valid option, you must choose a number! Valid options are: 1, 2, 3, 4')
                    continue

            while not rospy.is_shutdown():

                restart = raw_input('\nContinue expertiment? Answer with YES or NO: ') 
                rospy.loginfo('\nYou entered: '+ str(restart))

                if (restart) == 'yes' or (restart) == 'YES' or (restart) == 'Yes' or (restart) == 'Y' or (restart) == 'y':
                    break 

                elif (restart) == 'no' or (restart) == 'NO' or (restart) == 'No' or (restart) == 'N' or (restart) == 'n':
                    youBot_placing.start()
                    exit()

                else:
                    rospy.logerr('\nNot a valid option, you must choose between YES or NO!')
                    continue

        except KeyboardInterrupt:
            rospy.logerr('Process shutting down')
            exit()
            break
