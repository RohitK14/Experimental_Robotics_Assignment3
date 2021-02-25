#!/usr/bin/env python
# To run this file go to the src folder and type
# $ chmod +x state_behavior.py

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


home_fixed = Point()
home_fixed.x = rospy.get_param('home_x', 0)
home_fixed.y = rospy.get_param('home_y', 0)
tired_level = rospy.get_param('tireness_level', 2)

global flag_state
flag_state = 1


class Normal(smach.State):
    ##
    #   \brief __init__ initialises the Normal state in the smach_state
    #   \param outcomes are the possible transitions which are either it can go to normal ot play state
    #   \param input_keys These are possible input of the state
    #   \param output_keys These are possible outputs of the state.
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
        ##
        #   \brief In this execute() function, the robot randomly walks in the environment until it reaches the tired_level
        #   The lograndintic used here is that if we receive a command play in the middle, it reaches the
        #   last coordinate and that is published in the rostopic /moveToPose and shifts to Play state.
        #   Otherwise the robot goes to "Sleep" state after it gets tired
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
    ##
    #   \brief __init__ initialises the Sleep state in the smach_state
    #   \param outcomes are the possible transition is it can to normal state by wake_up transition
    #   \param input_keys These are possible input of the state
    #   \param output_keys These are possible outputs of the state.
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['wake_up'])
        self.client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)

    def execute(self, userdata):
        ##
        #   \brief In this execute() function, the robot goes to predefined home_fixed position
        #   The position is published in the topic /moveToPose
        #   The logic used here is that if we receive a command play in the middle, it reaches the
        #   last coordinate and that is published in the rostopic /moveToPose and shifts to Play state.
        #   Otherwise the robot goes to "Sleep" state after it gets tired

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
    ##
    #   \brief __init__ initializes the Play state with the outcome go_to_normal.
    #   \param  outcomes lists the possible transitions. From play we can go to normal state.
    #   \param input_keys It is the possible input of the state
    #   \pram output keys It is the possible output of the state
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['go_to_normal'])
        self.pubTarget = rospy.Publisher('/goTocmd', String, queue_size=1)

        self.client = actionlib.SimpleActionClient(
                '/move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.human = Point(x=-5, y=8)

    def execute(self, userdata):
        ##
        #   In this execute(), we implement play behavior.
        #   A random position is generated for a person. The robot goes to the person, waits for the gesture and
        #   and goes to gesture position.
        #   the robot goes and comes back to the gesture position and waits for another gesture position until
        #   it gets tired.
        #   At last the robot goes to Normal position.

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
