#!/usr/bin/env python3

import numpy as np
import rospy
import math
from roblib import *
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu,NavSatFix
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped


Glat, Glong, Gdepth, Gquat = 0, 0, 0, Quaternion()


### = - = - = - = - = - = - = - = - = - = - = - = - = # CALL BACK # = - = - = - = - = - = - = - = - = - = - = - = - = - = - = ###


def sub_euler(data):
    global Gquat
    Gquat = data.orientation

def sub_gps(data):
    global Glat, Glong
    
    Glat = data.latitude
    Glong = data.longitude

def sub_depth(data):
    global Gdepth
    Gdepth = data.data

def lat_long_toxy(lat, long, z, centre=(48.340777, -4.492595)):
    rayon = 6371000
    x = np.cos(centre[0])*(long - centre[1])
    y = lat - centre[0]
    return x, y, z

def xy_to_latlong(x, y, z, centre=(48.340777, -4.492595)):
    rayon = 6371000
    lat = y + centre[0]
    longi = x/np.cos(centre[0]) + centre[1]
    return lat, longi, z

def euler_from_quaternion(x, y, z, w):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


### = - = - = - = - = - = - = - = - = - = - = - = - = # ROS # = - = - = - = - = - = - = - = - = - = - = - = - = - = - = - = ###


class Obs(object):

    def __init__(self):
        rospy.init_node('Observer')

        rospy.Subscriber("/imu",Imu, sub_euler)
        rospy.Subscriber('/gps', NavSatFix, sub_gps)
        rospy.Subscriber('/depth', Float32, sub_depth)

        self.state_pub = rospy.Publisher('/state', PoseStamped,  queue_size=5)

    def send2controler(self):
        global Glat, Glong, Gdepth

        rate = rospy.Rate(10)
        x_old = 0

        while not rospy.is_shutdown():

            state = PoseStamped()
            state.header.stamp = rospy.Time.now()
            state.header.frame_id = "Observer"

            x, y, z = lat_long_toxy(Glat, Glong, Gdepth)
            v = (x - x_old)*10
            phi, theta, psi = euler_from_quaternion(Gquat.x, Gquat.y, Gquat.z, Gquat.w)            
            E = eulermat(phi, theta, psi)
            dp = matmul(E, array([[v], [0], [0]])).reshape(3,)
        
            state.pose.position.x = x + 1/10 * dp[0]
            state.pose.position.y = y + 1/10 * dp[1]
            state.pose.position.z = z + 1/10 * dp[2]
            state.pose.orientation = Gquat
            x_old = x

            # Estimate new lat long and depth of the robot
            Glat, Glong, Gdepth = xy_to_latlong(state.pose.position.x, state.pose.position.y, state.pose.position.z)

            self.state_pub.publish(state)

            rate.sleep()


if __name__ == "__main__":

    observer = Obs()
    observer.send2controler()
    rospy.spin()
