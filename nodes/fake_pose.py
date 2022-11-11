#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
from dora import Node

GOAL_LOCATION = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 8.0]])

node = Node()

start = time.time()

node.send_output("gps_waypoints", GOAL_LOCATION.tobytes())

for _, _, _ in node:
    # Wait next dora_input
    array = np.array([0, 0, 0, 0.0, 0.0, 0.0, 1.0])
    node.send_output("position", array.tobytes())

time.sleep(1)
