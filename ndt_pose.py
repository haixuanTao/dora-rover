#!/usr/bin/env python

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from dora import Node
import time
from scipy.spatial.transform import Rotation as R

node = Node()
initial_orientation = None
orientation = None


def imu_callback(data):

    global initial_orientation
    global orientation

    if initial_orientation is None:
        initial_orientation = R.from_quat(
            [
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w,
            ]
        )
        print(
            f"INITIAL ORIENTATION: {initial_orientation.as_euler('xyz', degrees=True)}"
        )
    orientation = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w,
    ]

    # node.send_output("imu", orientation.tobytes())


start = time.time()


def pose_callback(data):
    global start
    global initial_orientation
    global orientation
    position = np.array(
        [
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
        ]
    )
    if initial_orientation is None:
        return None

    position = initial_orientation.apply(position)
    position = np.concatenate([position, orientation])

    if time.time() - start > 0.5:
        node.send_output("position", position.tobytes())
        start = time.time()


rospy.init_node("listener", anonymous=True)
rospy.Subscriber("mavros/imu/data", Imu, imu_callback)
rospy.Subscriber("current_pose", PoseStamped, pose_callback)

rospy.spin()
