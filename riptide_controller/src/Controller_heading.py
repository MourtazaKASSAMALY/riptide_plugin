#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32,Int32,Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import WaypointList, Waypoint, State,CommandCode

from modele_terrain_v2 import *
from riptide import Riptide
from Drivers import *

from roblib import *

import math
import sys

Geuler_angles = (0, 0, 0)
Gpos = (0, 0, 0)
Gspeed = 3
gazebo_disabled = False


### = - = - = - = - = - = - = - = - = - = - = - = - = # Function # = - = - = - = - = - = - = - = - = - = - = - = - = - = - = ###


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

def waypoint_to_heading(waypoint):
    """convert a waypoint (x,y,z) to the desired heading and depth wanted"""

    global Gpos

    x, y, z = waypoint
    xsub, ysub, zsub = Gpos
    heading = 0
    if x != xsub: heading = arctan((y - ysub) / (x - xsub))
    dep = z
    return heading, dep

def valid_waypoint(waypoint, r):
    """return true if the waypoint is validated"""
    
    global Gpos

    x, y, z = waypoint
    xsub,ysub,zsub = Gpos

    if ((abs(x - xsub) < r) and (abs(y - ysub) < r) and (abs(z - zsub) < r)): return True
    else: return False

def next_waypoint(list_waypoint):
    """return next waypoint from the list of waypoints"""
    
    if len(list_waypoint) == 0:
        return waypoint_init
    else:
        return list_waypoint.pop(0)  # return first waypoint and update list_waypoints


### = - = - = - = - = - = - = - = - = - = - = - = - = # CALLBACKS # = - = - = - = - = - = - = - = - = - = - = - = - = - = - = ###


def sub_state(data):
    global Geuler_angles, Gpos

    x, y, z, w = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
    Geuler_angles = euler_from_quaternion(x, y, z, w)

    Gpos = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

def sub_pose(data):
    global Geuler_angles, Gpos

    x, y, z, w = data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w
    Geuler_angles = euler_from_quaternion(x, y, z, w)

    Gpos = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)


### = - = - = - = - = - = - = - = - = - = - = - = - = # ROS # = - = - = - = - = - = - = - = - = - = - = - = - = - = - = - = ###


class Control(object):

    def __init__(self):
        global gazebo_disabled

        rospy.init_node('Controller')
        rospy.loginfo("Initializing controller")

        rospy.Subscriber('/state', PoseStamped, sub_state)
        # rospy.Subscriber('riptide/pose_gt', Odometry, sub_pose)  # switch from sub_state (IMU) and sub_pose (exact pose from Gazebo)

        self.esc_pub = rospy.Publisher('/pololu/esc', Float32, queue_size=5)
        self.servo1_pub = rospy.Publisher('/pololu/servo1', Float32, queue_size=5)
        self.servo2_pub = rospy.Publisher('/pololu/servo2', Float32, queue_size=5)
        self.servo3_pub = rospy.Publisher('/pololu/servo3', Float32, queue_size=5)
        self.light_pub = rospy.Publisher('/pololu/light', Bool, queue_size=5)

        # Init Riptide
        Xinit_riptide = array([[0., 0., 0.,  # x, y, z
                                0., 0., 0.,  # phi, theta, psi
                                0.]]).T  # v

        self.submarine = Riptide(Xinit_riptide)

        ### Figure initialisation
        if gazebo_disabled: self.ax = plt.axes(projection='3d')


    def control(self, heading=0, depth=-1, f=10):
        global gazebo_disabled

        rate = rospy.Rate(f)

        while not rospy.is_shutdown():

            # Calculation of the command to follow current heading.
            state = self.submarine.X.flatten()
            state[0] = Gpos[0]
            state[1] = Gpos[1]
            state[2] = Gpos[2]

            self.submarine.X = state
            u = self.submarine.control(Gspeed, heading, depth, Geuler_angles, Gpos[2])

            #Publish the command
            u0, u1, u2, u3 = Float32(), Float32(), Float32(), Float32()

            u0.data = u[0]*200  # amplify the linear speed for Gazebo (value of 100 hundred needed to actually move forward)
            u1.data = u[1]
            u2.data = u[2]
            u3.data = u[3]
            
            self.esc_pub.publish(u0)
            self.servo1_pub.publish(u1)
            self.servo2_pub.publish(u2)
            self.servo3_pub.publish(u3)

            if gazebo_disabled:
                # Simulation of the evolution of the robot
                dX = self.submarine.evolX(u)
                self.submarine.X = self.submarine.X + (1 / f) * dX.flatten()

                self.ax.clear()
                self.ax.relim()
                draw_riptide(self.ax, self.submarine.X)

                self.ax.autoscale_view(True, True, True)
                self.ax.set_xlim(-20, 50)
                self.ax.set_ylim(-20, 50)
                self.ax.set_zlim(-30, 20)
                self.ax.set_xlabel('X axis, meters')
                self.ax.set_ylabel('Y axis, meters')
                self.ax.set_zlabel('Z axis, meters')
                pause(1/f)

            rate.sleep()


if __name__ == "__main__":

    controler = Control()

    target_heading = float(sys.argv[1])
    target_depth = float(sys.argv[2])
    controler.control(target_heading, target_depth)

    rospy.spin()
