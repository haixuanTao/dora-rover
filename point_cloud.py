#!/usr/bin/env python
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from dora import Node

node = Node()


def callback(data):
    gen = pc2.read_points(data, skip_nans=True)
    data = np.array(list(gen))
    node.send_output("lidar_pc", data.tobytes())


rospy.init_node("listener", anonymous=True)
rospy.Subscriber("velodyne_points", PointCloud2, callback)
rospy.spin()
