#!/usr/bin/env python
# # To run this file go to the src folder and type
# $ chmod +x state_behavior.py

"""
    \package exp_assignment3
    \file state_behavior.py
    \brief This file contains the behaviour of a of the finite state machine.
    \author Rohit Kumar
    \date 25/02/2021

    \param [in] home_x
    \param [in] home_y
    \param [in] tireness_level

    Returns:
        [Finite state diagram]: [Differnet states can be visualised with the help of smach_viewer]
    """
import rospy
import smach
import smach_ros
import time
import random
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import actionlib
import actionlib.msg

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Python libs
import sys
import time
# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
"""
Parameters:
    [home_x]: [The launch file describes the home position of the robot. ]
    [tireness_level]: [It tells us the number of times a task can be performed before the fatigue.]
"""
home_fixed = Point()
home_fixed.x = rospy.get_param('home_x', 0)
home_fixed.y = rospy.get_param('home_y', 0)
tired_level = rospy.get_param('tireness_level', 2)

global flag_state
flag_state = 1


class Normal(smach.State):
    """
    \brief __init__ initialises the Normal state in the smach_state
       \param outcomes are the possible transitions which are either it can go to normal ot play state
       \param input_keys These are possible input of the state
       \param output_keys These are possible outputs of the state.

    Subscribers:
    	sub_human: subscriber (std_msgs.String) to /command
		subscribe to get the command from human client to enter the PLAY state
    Actions:
    	client: Client for action /move_base
		The client calls the action sever to move the robot to the specified target on the plane.
		goal: geometry_msgs.PoseStamped
		result: geometry_msgs.Pose
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['go_sleep', 'start_play'],
                             input_keys=['normal_tired_counter_in'],
                             output_keys=['normal_tired_counter_out'])

        self.client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goal = MoveBaseGoal()

        self.sub_human = rospy.Subscriber("/command", String, self.cmdCallback)
        self.found_image = 0
        self.counter = 0
        self.human = "none"

    def cmdCallback(self, data):
        self.human = data.data

    def execute(self, userdata):
        """
        \brief In this execute() function, the robot randomly walks in the environment until it reaches the tired_level
           The logarithm used here is that if we receive a command play in the middle, it reaches the
           last coordinate and then shifts to the Play state.
         Otherwise the robot goes to "Sleep" state after it gets tired with respect to tireness_level
         The goals are cancelled as soon as the play command is received.
         Returns:
            Sleep state - This is returned after tiring
            Play state - This is returned after the command is received.
        """

        userdata.normal_tired_counter_out = 0
        self.counter = 1
        while not rospy.is_shutdown():
            rospy.loginfo('Executing state Normal')
            global flag_state
            flag_state = 1
            if self.human == "play":
                print('Human called. Switching to play mode')
                self.client.cancel_all_goals()
                return 'start_play'

            # Random positions for the robot to move
            self.goal.target_pose.header.frame_id = "map"
            self.goal.target_pose.header.stamp = rospy.get_rostime()

            self.goal.target_pose.pose.position.x = random.randint(-6, 6)
            self.goal.target_pose.pose.position.y = random.randint(-8, 8)
            self.goal.target_pose.pose.position.z = 0.0
            self.goal.target_pose.pose.orientation.w = 1.0
            print("Robot going to: ", self.goal.target_pose.pose.position.x,
                  ",", self.goal.target_pose.pose.position.y)
            self.client.send_goal(self.goal)
            self.client.wait_for_result()
            self.client.get_result()
            userdata.normal_tired_counter_out = userdata.normal_tired_counter_in + 1
            self.counter = self.counter + 1
            if self.counter >= tired_level:
                print("Robot is tired. Going to sleep...")
                return 'go_sleep'


# define state Sleep
class Sleep(smach.State):
    """\brief __init__ initialises the Sleep state in the smach_state
       \param outcomes are the possible transition is it can to normal state by wake_up transition

    Args:
        smach ([state]): The state of the finite machine is taken

    Action server: 
            client: Action server to mobve the base to specified coordinates in the world.
    """
       
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wake_up'])
        self.client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)

    def execute(self, userdata):
        """\brief In this execute() function, the robot goes to predefined home_fixed position
           
           The robot goes to "Sleep" state after it gets tired. It stays for 10s and then shifts back to Normal state.

        Returns:
            [wake_up]: transition state 
        """
           

        rospy.loginfo('Executing state Sleep')
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = home_fixed.x
        goal.target_pose.pose.position.y = home_fixed.y
        print("Sleeping at location: ", home_fixed.x, ",", home_fixed.y)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        self.client.get_result()
        rospy.sleep(10)
        print('I am awake now')
        return 'wake_up'


# define state Play
class Play(smach.State):
    """   \brief __init__ initializes the Play state with the outcome go_to_normal.
       \param  outcomes lists the possible transitions. From play we can go to normal state.

    The human is considered to be stationary at point(-5,8). The robot comes to the human and then
    follows his commands and returns back to the human.
    
    Args:
        smach ([state]): This state is responsible for the Play behavior of the robot

    Returns:
        [go_to_normal]: when the seach is over returns back to normal state.
    """
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['go_to_normal'])
        self.pubTarget = rospy.Publisher('/goTocmd', String, queue_size=1)

        self.client = actionlib.SimpleActionClient(
                '/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.human = Point(x=-5, y=8)

    def execute(self, userdata):
        """
           In this execute(), we implement play behavior.
          A fixed position is set for a person. The robot goes to the person, waits for the gesture and
           and goes to gesture position.
           the robot goes and comes back to the gesture position and waits for another gesture position until
           it gets tired.
           At last the robot goes to Normal position.

        Returns:
            [go_to_normal]: transition state to switch back to Normal state
                    """
        

        rospy.loginfo('Executing state Play')
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose.position.x = self.human.x
        goal.target_pose.pose.position.y = self.human.y
        self.client.send_goal(goal)
        self.client.wait_for_result()
        self.client.get_result()
        n_play = random.randint(2, 3)
        i = 1
        while not rospy.is_shutdown():
            room = raw_input('Which room: ')
            print('Lets go to ' + room)
            if i < n_play:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.get_rostime()
                goal.target_pose.pose.position.x = random.randint(-6, 6)
                goal.target_pose.pose.position.y = random.randint(-8, 8)
                self.client.send_goal(goal)
                self.client.wait_for_result()
                self.client.get_result()
            else:
                print('Playing done')
                return 'go_to_normal'


# main
def main():
    rospy.init_node('state_behavior')

    random.seed()

    # Create a SMACH state machine
    sm = smach.StateMachine(
        outcomes=['Behaviours interface for sleep, normal, play and find'])

    sm.userdata.tireness = 0
    sm.userdata.person = Point()
    # Open the container
    """Generating the finite state machine with NORMAL, SLEEP and PLAY state.
    """
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(),
                               transitions={'go_sleep': 'SLEEP',
                                            'start_play': 'PLAY'},
                               remapping={'normal_tired_counter_in': 'tireness',
                                          'normal_tired_counter_out': 'tireness'})
        smach.StateMachine.add('SLEEP', Sleep(),
                               transitions={'wake_up': 'NORMAL'})

        smach.StateMachine.add('PLAY', Play(),
                               transitions={'go_to_normal': 'NORMAL'})
    sis = smach_ros.IntrospectionServer('robot_behavior', sm, '/SM_ROOT')
    sis.start()
    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    cv2.destroyAllWindows()
    sis.stop()


if __name__ == '__main__':
    main()
