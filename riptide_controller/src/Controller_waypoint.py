#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32,Int32,Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped
from mavros_msgs.msg import WaypointList, Waypoint, State,CommandCode

from roblib import *
from modele_terrain_v2 import *
from riptide import Riptide
from Drivers import *

import math
import time


Geuler_angles = (0, 0, 0)
Gpos = (0, 0, 0)
Gwaypoints = []
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

def waypoint_to_heading(waypoint, submarine):
    """convert a waypoint (x,y,z) to the desired heading and depth wanted"""

    global Gpos

    x, y, z = waypoint
    state = submarine.X.flatten()
    xsub, ysub, zsub = state[0], state[1], state[2]

    heading = 0
    if x != xsub:
        heading = np.arctan2((x-xsub), (y-ysub))
    dep = z
    return heading, dep


def valid_line(w1, w2, submarine):
    # global Gpos

    xs, ys, zs = submarine.X[0], submarine.X[1], submarine.X[2]
    pos = np.array((xs, ys, zs)).flatten()
    p = np.vdot((np.array(w2) - np.array(w1)),(np.array(pos) - np.array(w2)))

    return p >= 0

def valid_waypoint(waypoint, r):
    """return true if the waypoint is validated"""
    global Gpos

    x, y, z = waypoint
    xsub, ysub, zsub = Gpos

    if ((abs(x - xsub) < r) and (abs(y - ysub) < r) and (abs(z - zsub) < r)): return True
    else: return False

def next_waypoint(list_waypoint, waypoint_init):
    """return next waypoint from the list of waypoints"""
    
    if len(list_waypoint) == 0:
        # retourne au debut
        return waypoint_init
    else:
        return list_waypoint.pop(0)  # return first waypoint and update list_waypoints


### = - = - = - = - = - = - = - = - = - = - = - = - = # CALL BACK # = - = - = - = - = - = - = - = - = - = - = - = - = - = - = ###


def sub_state(data):
    global Geuler_angles, Gpos

    x, y, z, w = data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w
    Geuler_angles = euler_from_quaternion(x, y, z, w)
    
    Gpos = (data.pose.position.x, data.pose.position.y, data.pose.position.z)

def sub_target(data):
    global Gwaypoints

    Gwaypoints = []
    for k in data.waypoints: Gwaypoints.append((k.x_lat, k.y_long, k.z_alt))

def lat_long_toxy(point):
    lat, long, z = point
    centre = [48.340777, -4.492595]
    rayon = 6371000
    x = cos(centre[0])*(long - centre[1])
    y = (lat - centre[0])
    return x, y, z

def xy_to_latlong(point):
    x, y, z = point
    centre = [48.340777, -4.492595]
    rayon = 6371000
    lat = y + centre[0]
    long = x/cos(centre[0]) + centre[1]
    return lat, long, z


### = - = - = - = - = - = - = - = - = - = - = - = - = # ROS # = - = - = - = - = - = - = - = - = - = - = - = - = - = - = - = ###


class Control(object):

    def __init__(self):
        global gazebo_disabled

        rospy.init_node('Controleur')
        rospy.loginfo("Initializing controler")

        rospy.Subscriber('/state', PoseStamped, sub_state)
        rospy.Subscriber('/waypoints', WaypointList, sub_target)

        self.esc_pub = rospy.Publisher('/pololu/esc', Float32, queue_size=5)
        self.servo1_pub = rospy.Publisher('/pololu/servo1', Float32, queue_size=5)
        self.servo2_pub = rospy.Publisher('/pololu/servo2', Float32, queue_size=5)
        self.servo3_pub = rospy.Publisher('/pololu/servo3', Float32, queue_size=5)
        self.light_pub = rospy.Publisher('/pololu/light', Bool, queue_size=5)

        # Init Riptide

        time.sleep(3)
        self.waypoint_init = Gwaypoints[0]
        print(self.waypoint_init)
        self.list_waypoint = Gwaypoints

        self.waypoint = lat_long_toxy(next_waypoint(self.list_waypoint,self.waypoint_init))
        self.waypoint2 = lat_long_toxy(next_waypoint(self.list_waypoint,self.waypoint_init))

        Xinit_riptide = array([[self.waypoint_init[0], self.waypoint_init[1], self.waypoint_init[2],  # x, y, z
                                0., 0., 0.,  # phi, theta, psi
                                0.]]).T  # v

        self.submarine = Riptide(Xinit_riptide)

        ### Figure initialisation
        if gazebo_disabled: self.ax = plt.axes(projection='3d')

    def control(self, f=10):
        global gazebo_disabled

        rate = rospy.Rate(f)

        while not rospy.is_shutdown():

            if valid_line(self.waypoint, self.waypoint2, self.submarine):
                self.waypoint = self.waypoint2
                self.waypoint2 = lat_long_toxy(next_waypoint(self.list_waypoint, self.waypoint_init))
                print("New waypoint:", self.waypoint2)

            heading, depth = waypoint_to_heading(self.waypoint2, self.submarine)

            # Calculation of the command to follow current heading.
            u = self.submarine.control(Gspeed, heading, depth, Geuler_angles, Gpos[2])

            # Publish the command
            u0, u1, u2, u3 = Float32(), Float32(), Float32(), Float32()

            u0.data = u[0][0]
            u1.data = u[1][0]
            u2.data = u[2][0]
            u3.data = u[3][0]

            self.esc_pub.publish(u0)
            self.servo1_pub.publish(u1)
            self.servo2_pub.publish(u2)
            self.servo3_pub.publish(u3)

            if gazebo_disabled:
                dX = self.submarine.evolX(u)
                self.submarine.X = self.submarine.X + (1 / f) * dX

                self.ax.clear()
                self.ax.relim()
                state_plot = self.submarine.X
                state_plot[0] = state_plot[0]*180/np.pi
                state_plot[1] = state_plot[1]*180/np.pi

                draw_riptide(self.ax, state_plot)

                self.ax.scatter(self.waypoint[0],self.waypoint[1],self.waypoint[2],color='black')
                self.ax.scatter(self.waypoint2[0], self.waypoint2[1], self.waypoint2[2], color='red')
                self.ax.autoscale_view(True, True, True)

                self.ax.set_zlim(-30, 20)
                self.ax.set_xlabel('X axis, meters')
                self.ax.set_ylabel('Y axis, meters')
                self.ax.set_zlabel('Z axis, meters')
                pause(1/f)

            rate.sleep()


if __name__ == "__main__":

    controler = Control()
    controler.control(display=False)

    rospy.spin()
