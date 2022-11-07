#!/home/nvidia/Documents/dora-rover/venv/bin/python3
import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from dora import Node

node = Node()


def callback(data):
    gen = pc2.read_points(data, skip_nans=True)
    data = np.array(list(gen))
    point_cloud = data[:, :3]

    # To camera coordinate
    # The latest coordinate space is the velodyne space.
    point_cloud = np.dot(
        point_cloud,
        np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]]),
    )
    point_cloud = point_cloud[np.where(point_cloud[:, 2] > 0.1)]
   # inds0 = np.random.choice(point_cloud.shape[0], 5000, replace=False)
   # point_cloud = point_cloud[inds0]

    node.send_output("lidar_pc", point_cloud.tobytes())


rospy.init_node("listener", anonymous=True)
rospy.Subscriber("velodyne_points", PointCloud2, callback)
rospy.spin()
