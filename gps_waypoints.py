from typing import Callable

import numpy as np

from dora_utils import DoraStatus, closest_vertex
import os
import sys

sys.path.append(os.getcwd())


# Planning general
NUM_WAYPOINTS_AHEAD = 0
GOAL_LOCATION = np.array([[3, 3]])


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.position = []
        self.waypoints = []
        self.target_speeds = []

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):

        self.position = np.frombuffer(dora_input["data"])

        if len(self.waypoints) != 0:
            (index, _) = closest_vertex(
                self.waypoints,
                np.array([self.position[:2]]),
            )

            self.waypoints = self.waypoints[index : index + NUM_WAYPOINTS_AHEAD]
            self.target_speeds = self.target_speeds[index : index + NUM_WAYPOINTS_AHEAD]

        if len(self.waypoints) < NUM_WAYPOINTS_AHEAD / 2:

            self.waypoints = GOAL_LOCATION
            self.target_speeds = np.array([2.0] * len(waypoints))

        waypoints_array = np.concatenate(
            [self.waypoints.T, self.target_speeds.reshape(1, -1)]
        )

        send_output(
            "gps_waypoints", waypoints_array.tobytes(), dora_input["metadata"]
        )  # World coordinate

        return DoraStatus.CONTINUE
