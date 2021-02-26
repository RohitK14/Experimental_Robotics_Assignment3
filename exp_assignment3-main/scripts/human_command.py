#!/usr/bin/env python
""" 
    \package exp_assignment3
    \file human_command.py
    \brief This file contains the behaviour of a human to interact with the robot.
    \author Rohit Kumar
    \date 25/02/2021
    """
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import actionlib
import actionlib.msg

import time
import random


def time_counter(seconds):
    """A time_counter function to wait for the specfied number of time.

    Args:
        seconds ([integer]): [This argument is used to calculate the time passed when comapared to current time]
    """
    start_time = time.time()
    my_time = 0
    while (my_time < seconds):
        my_time = time.time() - start_time


global arrived
arrived = False


def Callback(data):
    """This is a callback to receive data from the ros topic /waitForRobot
    """
    global arrived
    arrived = data.data


def main():
    """The main function tries to emulate the human client by saying the command play.
    The command play is sent after random time. 

    Publishers:
            pub: /command std_msgs.String [It send the command to switch to play state]
    Subscribers:
            sub: /waitForRobot std_msgs.Bool [It checks whether the robot has arrived to the human]

    """
    rospy.init_node('human_command')

    pub = rospy.Publisher("/command", String, queue_size=10)
    sub = rospy.Subscriber("/waitForRobot", Bool, Callback)

    text = "play"
    time_counter(20)

    while True:
        pub.publish(text)
        time_bw_calls = random.randint(50,60)
        global arrived
        if arrived == True:
            time_counter(time_bw_calls)
    
if __name__ == "__main__":
    main()
