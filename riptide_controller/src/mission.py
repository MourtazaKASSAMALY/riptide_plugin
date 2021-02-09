#!/usr/bin/env python3

import numpy as np
import json
from mavros_msgs.msg import WaypointList, Waypoint, State,CommandCode
from mavros_msgs.srv import WaypointPush, WaypointClear, WaypointSetCurrent,SetMode
import rospy
import sys
import os

# see enum MAV_FRAME

FRAME_GLOBAL = 0
FRAME_LOCAL_NED = 1
FRAME_MISSION = 2
FRAME_GLOBAL_REL_ALT = 3
FRAME_LOCAL_ENU = 4
NAV_TAKEOFF = 22

# Parse *.json file to read Waypoints (*.json file could be generated from QGIS using a plugin)

class plannif(object):

    def __init__(self):
        rospy.init_node('Planificateur')
        rospy.loginfo("Initializing Planificateur")

        rospy.loginfo("publish waypoints")
        self.waypoint_pub = rospy.Publisher('/waypoints', WaypointList,  queue_size=5)

    def read_json(self,filename):
        with open(filename) as json_data: data_dict = json.load(json_data)

        waypoint_list = WaypointList()
        wp = Waypoint()
        wp.command = CommandCode.NAV_TAKEOFF
        waypoint_list.waypoints.append(wp)
        wp.is_current = True
        c = 0
        for phase in (data_dict["phases"]):
            wp = Waypoint()
            wp.is_current = 0
            wp.frame = 3
            if phase['type'] == 'Waypoint':
                wp.param1 = 0
                wp.command = CommandCode.NAV_WAYPOINT
                wp.autocontinue = True
                wp.x_lat = phase['point']['latitude']
                wp.y_long = phase['point']['longitude']
                wp.z_alt = -phase['depth']
                waypoint_list.waypoints.append(wp)
            
            if phase['type'] == 'Path':
                for track in phase['tracks']:
                    wp.param1 = 0
                    wp.command = CommandCode.NAV_WAYPOINT
                    wp.autocontinue = True
                    wp.x_lat = track['start_point']['latitude']
                    wp.y_long = track['start_point']['longitude']
                    wp.z_alt = track['altitude']
                    waypoint_list.waypoints.append(wp)
                    wp.command = CommandCode.NAV_SPLINE_WAYPOINT
                    wp.autocontinue = True
                    wp.x_lat = track['end_point']['latitude']
                    wp.y_long = track['end_point']['longitude']
                    wp.z_alt = track['altitude']
                    waypoint_list.waypoints.append(wp)
            
            if phase['type'] == 'Dive':
                wp.command = CommandCode.NAV_CONTINUE_AND_CHANGE_ALT
                wp.autocontinue = True
                wp.param1 = 2  # to dive and 1 to climb
                wp.z_alt = -phase['depth']
                waypoint_list.waypoints.append(wp)
            
            if phase['type'] == 'Ascent':
                wp.command = CommandCode.NAV_CONTINUE_AND_CHANGE_ALT
                wp.autocontinue = True
                wp.param1 = 1
                wp.z_alt = 0
                waypoint_list.waypoints.append(wp)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            self.waypoint_pub.publish(waypoint_list)
            rate.sleep()

if __name__ == "__main__":
    print("Current working directory: ", os.getcwd())

    p = plannif()
    p.read_json(sys.argv[1])
    rospy.spin()
