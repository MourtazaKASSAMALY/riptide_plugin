#!/usr/bin/env python3

from __future__ import division
import rospy
from sensor_msgs.msg import Imu,NavSatFix
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, WrenchStamped
from std_msgs.msg import Float32


MAX_VAL = 32767
MIN_VAL = -32768

# Publish dummy sensor values to the controller for when Gazebo is not running

def normalize(val, min_val, max_val):
    val_between_0_and_1 = (val - min_val) / (abs(min_val) + abs(max_val))
    val_between_minus_1_and_1 = (val_between_0_and_1 * 2.0) - 1.0
    return val_between_minus_1_and_1


def fix(val):
    return normalize(val, MIN_VAL, MAX_VAL)


def create_quaternion(q1, q2, q3, q4):
    q = Quaternion()
    q.x = fix(q2)
    q.y = fix(q3)
    q.z = fix(q4)
    q.w = fix(q1)
    return q


def create_vector3(groll, gpitch, gyaw):
    v = Vector3()
    v.x = groll / 1000.0
    v.y = gpitch / 1000.0
    v.z = gyaw / 1000.0
    return v


class Drivers(object):
    
    def __init__(self):
        
        rospy.loginfo("Initializing imu publisher")
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=5)

        rospy.loginfo("Initializing GPS publisher")
        self.gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=5)

        rospy.loginfo("Initializing depth publisher")
        self.depth_pub = rospy.Publisher('/depth', Float32, queue_size=5)

    def init_values(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            i = Imu()
            i.header.stamp = rospy.Time.now()
            i.header.frame_id = 'Riptide_INS'
            i.orientation = create_quaternion(0, 0, 0, 0)
            i.angular_velocity = create_vector3(1, 2, 3)
            self.imu_pub.publish(i)

            gpsmsg = NavSatFix()
            gpsmsg.header.stamp = rospy.Time.now()
            gpsmsg.header.frame_id = "gps"
            gpsmsg.latitude = float(0) # /!\ change here
            gpsmsg.longitude = float(0) # /!\ change here
            self.gps_pub.publish(gpsmsg)

            depth = Float32()
            depth.data = 1 # /!\ change here
            self.depth_pub.publish(Float32(data=1))

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('Drivers')
    rate = rospy.Rate(10)
    data_drivers = Drivers()
    data_drivers.init_values()
