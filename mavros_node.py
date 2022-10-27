#!/usr/bin/env python
# license removed for brevity
import rospy
from mavros_msgs.msg import PositionTarget, OverrideRCIn
from geometry_msgs.msg import TwistStamped
from dora import Node
import numpy as np
import time
node = Node()

def talker():
    pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    for input_id, value, metadata in node:
        [angle] = np.frombuffer(value)
        print(f"vel: {angle}")
        target = OverrideRCIn()
        print( int((angle + np.pi)  / (2* np.pi) * 2000))
        target.channels[0] =int(( -angle + np.pi)  / (2* np.pi) * 2000)
        # target.channels[2] = 100
       # target = PositionTarget()
       # target.coordinate_frame = 9
       # target.header.stamp = rospy.get_rostime()
       # target.type_mask = int("110111111100",2)
       # target.position.x = 0.9
       # target.position.y = -0.9
       # target.velocity.x = 0.1
       # target.velocity.y = -0.1
        # target.yaw = yaw
        pub.publish(target)

        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass