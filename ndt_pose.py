#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from dora import Node
import time

node = Node()

def imu_callback(data):
    imu_data = np.array(
        [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w,
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z,
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
        ]
    )

    node.send_output("imu", imu_data.tobytes())

start = time.time()

def pose_callback(data):
    global start
    position = np.array(
        [
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w,
        ]
    )


    if time.time() - start > 1:
        node.send_output("position", position.tobytes())
        start = time.time()


rospy.init_node("listener", anonymous=True)
rospy.Subscriber("mavros/imu/data", Imu, imu_callback)
rospy.Subscriber("current_pose", PoseStamped, pose_callback)

rospy.spin()
