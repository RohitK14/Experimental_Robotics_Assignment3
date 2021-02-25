#!/usr/bin/env python

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
import rospy

# Ros Messages

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from gazebo_msgs.msg import LinkState
from tf import transformations
import math
import actionlib
import actionlib.msg

from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global rooms
kitchen = Point(x=0, y=0)
closet = Point(x=0, y=0)
livingRoom = Point(x=0, y=0)
entrance = Point(x=0, y=0)
bathroom = Point(x=0, y=0)
bedroom = Point(x=0, y=0)
rooms = [kitchen, closet, livingRoom, entrance, bathroom, bedroom]

global colors
colors = ['yellow', 'red', 'green', 'blue', 'magenta', 'black']

blue_low = (100, 50, 50)
blue_high = (130, 255, 255)
red_low = (0, 50, 50)
red_high = (5, 255, 255)
green_low = (50, 50, 50)
green_high = (70, 255, 255)
yellow_low = (25, 50, 50)
yellow_high = (35, 255, 255)
magenta_low = (125, 50, 50)
magenta_high = (150, 255, 255)
black_low = (0, 0, 0)
black_high = (5, 50, 50)

color_Lower = [
    yellow_low,
    red_low,
    green_low,
    blue_low,
    magenta_low,
    black_low]
color_Higher = [
    yellow_high,
    red_high,
    green_high,
    blue_high,
    magenta_high,
    black_high]

VERBOSE = False


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('image_feature', anonymous=True)
     # topic where we publish
        self.image_pub = rospy.Publisher("/camera1/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel",
                                       Twist, queue_size=1)
        self.pubLocation = rospy.Publisher("/location", Point, queue_size=10)
        self.pubColor = rospy.Publisher("/color_detect", String, queue_size=10)
        # subscribed Topic
        self.subscriber = rospy.Subscriber(
            "camera1/image_raw/compressed",
            CompressedImage,
            self.callback,
            queue_size=1)
        self.sub_color = rospy.Subscriber(
            "/goTocmd", String, self.colCallback, queue_size=1)

        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odoCallback)

        self.client = actionlib.SimpleActionClient(
            '/move_base', MoveBaseAction)

        self.color = 'none'
        self.explore = 1
        self.position = Point()

    def odoCallback(self, data):
        self.position = data.pose.pose.position

    def colCallback(self, data):
        if data.data == 'entrance':
            self.color = 'blue'
        if data.data == 'closet':
            self.color = 'red'
        if data.data == 'livingRoom':
            self.color = 'green'
        if data.data == 'kitchen':
            self.color = 'yellow'
        if data.data == 'bathroom':
            self.color = 'magenta'
        if data.data == 'bedroom':
            self.color = 'black'
        self.explore = 0

    def callback(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        for i in range(0, 6):

            mask = cv2.inRange(hsv, color_Lower[i], color_Higher[i])
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            #cv2.imshow('mask', mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            center = None
            # only proceed if at least one contour was found
            if len(cnts) > 0 and rooms[i].x == 0 and rooms[i].y == 0:
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid

                self.client.cancel_all_goals()

                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                # only proceed if the radius meets a minimum size
                if radius > 10:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    vel = Twist()
                    vel.angular.z = 0.002*(center[0]-400)
                    vel.linear.x = -0.01 * (radius - 200)
                    self.vel_pub.publish(vel)
                    if(abs(radius - 100) < 2):
                        print('Close to the ball')
                        rooms[i] = self.position
                        # self.pubLocation.publish(self.position)
                        # self.pubColor.publish(self.color)

                #else:
                #    vel = Twist()
                #    vel.linear.x = 0.05
                #    self.vel_pub.publish(vel)

            #else:
            #    vel = Twist()
            #    vel.angular.z = 0.5
            #    self.vel_pub.publish(vel)

            cv2.imshow('window', image_np)
            cv2.waitKey(2)

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
