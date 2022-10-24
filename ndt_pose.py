import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
import numpy as np
#from dora import Node

#node = Node()

def callback(data):
    position = [
        data.pose.position.x,
        data.pose.position.y,
        data.pose.position.z,
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w,
    ]
   # node.send_output("", data.tobytes())
    
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("os1_points", PoseStamped, callback)
rospy.spin()