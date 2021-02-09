#!/usr/bin/env python
# coding: utf-8

import rospy
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Int32, Float32, Bool
from sensor_msgs.msg import NavSatFix, Imu, FluidPressure
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

# This ROS Node remaps topics between Gazebo and the Controller
# Controller was designed with topics names according to the real robot
# Gazebo uses different topic names and message types for their sensors plugins

# --- Real robot

# Receive GNSS data on topic /gps with type NavSatFix (geometry_msgs)
# Receive IMU data on topic /imu with type Imu (geometry_msgs)
# Receive pressure data on topic /depth with type Float32 (std_msgs)
# Send thruster input on topic /pololu/esc with type Float32
# Send fin0 input on topic /pololu/servo1 with type Float32
# Send fin1 input on topic /pololu/servo2 with type Float32
# Send fin2 input on topic /pololu/servo3 with type Float32

# --- Gazebo

# Send GNSS data on topic /gps with type NavSatFix (geometry_msgs)
# Send IMU data on topic /imu with type Imu (geometry_msgs)
# Send pressure data on topic /depth with type FluidPressure (geometry_msgs)
# Receive thruster input on topic riptide/thrusters/0/input with type FloatStamped (uuv_gazebo_ros_plugins_msgs)
# Receive fin0 input on topic riptide/fins/0/input with type FloatStamped (uuv_gazebo_ros_plugins_msgs)
# Receive fin1 input on topic riptide/fins/1/input with type FloatStamped (uuv_gazebo_ros_plugins_msgs)
# Receive fin2 input on topic riptide/fins/2/input with type FloatStamped (uuv_gazebo_ros_plugins_msgs)

# ----------------------- Global publishers -----------------------

# Flag needed because Gazebo always send GNSS data, even if the AUV is underwater 
is_submerged = False

rospy.init_node('SimulationInterface', anonymous=False)
rate = rospy.Rate(10)  # 10 Hz

pub_gps = rospy.Publisher('gps', NavSatFix, queue_size=10)  # publish to controller
pub_imu = rospy.Publisher('imu', Imu, queue_size=10)  # publish to controller
pub_depth = rospy.Publisher('depth', Float32, queue_size=10)  # publish to controller

pub_thruster = rospy.Publisher('riptide/thrusters/0/input', FloatStamped, queue_size=10)  # publish to Gazebo
pub_fin0 = rospy.Publisher('riptide/fins/0/input', FloatStamped, queue_size=10)  # publish to Gazebo
pub_fin1 = rospy.Publisher('riptide/fins/1/input', FloatStamped, queue_size=10)  # publish to Gazebo
pub_fin2 = rospy.Publisher('riptide/fins/2/input', FloatStamped, queue_size=10)  # publish to Gazebo


# ----------------------- Callback for ROS Topics -----------------------


def callback_gps(msg):  # receive GNSS from Gazebo and republish it to the controller
	global is_submerged
	if not is_submerged: pub_gps.publish(msg)

def callback_submerged(msg):  # receive from Gazebo and update flag
	global is_submerged
	is_submerged = msg.data

def callback_imu(msg): pub_imu.publish(msg)  # receive IMU from Gazebo and republish it to the controller

def callback_pressure(msg): 
	depth = (101325. - msg.fluid_pressure*1000.) / 9806.38  # (Patmosphere - fluid_pressure from kPa to Pa)/(how much Pa per meter)
	pub_depth.publish(Float32(data=depth))  # receive pressure from Gazebo and republish it as depth to controller

def callback_esc(msg):  # receive thruster input from controller and republish it to Gazebo
	msgstamped = FloatStamped()
	msgstamped.header.stamp = rospy.Time.now()
	msgstamped.data = float(msg.data)
	pub_thruster.publish(msgstamped)

def callback_servo1(msg):  # receive fin0 input from controller and republish it to Gazebo
	msgstamped = FloatStamped()
	msgstamped.header.stamp = rospy.Time.now()
	msgstamped.data = float(msg.data)
	pub_fin0.publish(msgstamped)

def callback_servo2(msg):  # receive fin1 input from controller and republish it to Gazebo
	msgstamped = FloatStamped()
	msgstamped.header.stamp = rospy.Time.now()
	msgstamped.data = float(msg.data) 
	pub_fin1.publish(msgstamped)
	
def callback_servo3(msg):  # receive fin2 input from controller and republish it to Gazebo
	msgstamped = FloatStamped()
	msgstamped.header.stamp = rospy.Time.now()
	msgstamped.data = float(msg.data) 
	pub_fin2.publish(msgstamped)

 
# ----------------------- Main program -----------------------


if __name__ == '__main__':
	
	rospy.Subscriber("riptide/gps", NavSatFix, callback_gps)  # receive from Gazebo
	rospy.Subscriber("riptide/imu", Imu, callback_imu)  # receive from Gazebo
	rospy.Subscriber("riptide/pressure", FluidPressure, callback_pressure)  # receive from Gazebo
	rospy.Subscriber("/riptide/is_submerged", Bool, callback_submerged)  # receive from Gazebo

	rospy.Subscriber("pololu/esc", Float32, callback_esc)  # receive from controller
	rospy.Subscriber("pololu/servo1", Float32, callback_servo1)  # receive from controller
	rospy.Subscriber("pololu/servo2", Float32, callback_servo2)  # receive from controller
	rospy.Subscriber("pololu/servo3", Float32, callback_servo3)  # receive from controller

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
