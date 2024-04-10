#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf.transformations as tf

odom_publisher = rospy.Publisher('/yaw_odom', Float64, queue_size=10)
imu_publisher = rospy.Publisher('/yaw_imu', Float64, queue_size=10)

def callback_odom(msg: Odometry):
    pose=msg.pose.pose
    orientation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(orientation)
    
    odom_publisher.publish(Float64(yaw))

def imu_callback(imu_msg):
    orientation = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
    roll, pitch, yaw = tf.euler_from_quaternion(orientation)
    
    imu_publisher.publish(Float64(yaw))

def main():
    rospy.init_node('yaw_calculator')

    # 订阅两个不同topic的姿态信息
    rospy.Subscriber('/imu', Imu, imu_callback)
    rospy.Subscriber('/odom_chassis', Odometry, callback_odom)

    rospy.spin()

if __name__ == '__main__':
    main()
