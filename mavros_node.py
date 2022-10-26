#!/usr/bin/env python
# license removed for brevity
import rospy
from mavros_msgs.msg import PositionTarget
from dora import Node
import numpy as np

node = Node()

def talker():
    pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    for _ in range(1000):
        input_id, value, metadata = node.next()
        [vx, vy, vz, yaw] = np.frombuffer(value)
        print(f"vel: {vx, vy, vz, yaw}")
        target = PositionTarget()
        target.coordinate_frame = 7
        target.type_mask = 3527
        target.velocity.x = vx
        target.velocity.y = vy
        # target.yaw = yaw
        pub.publish(target)
        # rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass