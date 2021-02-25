#!/usr/bin/env python

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
	""" Function to wait the specified seconds
	"""
	start_time = time.time()
	my_time = 0
	while (my_time < seconds):
		my_time = time.time() - start_time


global arrived
arrived = False


def Callback(data):
	global arrived
	arrived = data.data


def main():

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
