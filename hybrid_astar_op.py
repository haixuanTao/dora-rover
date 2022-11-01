from typing import Callable
import time
import numpy as np
from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper import (
    apply_hybrid_astar,
)

from dora_utils import DoraStatus, closest_vertex, pairwise_distances
from scipy.spatial.transform import Rotation as R

# Hybrid ASTAR
STEP_SIZE_HYBRID_ASTAR = 1.0
MAX_ITERATIONS_HYBRID_ASTAR = 2000
COMPLETION_THRESHOLD = 1
ANGLE_COMPLETION_THRESHOLD = 4
RAD_STEP = 0.1
RAD_UPPER_RANGE = 4
RAD_LOWER_RANGE = 4
OBSTACLE_CLEARANCE_HYBRID_ASTAR = 0.2
LANE_WIDTH_HYBRID_ASTAR = 5.0
RADIUS = 0.1
CAR_LENGTH = 0.6
CAR_WIDTH = 0.3

OBSTACLE_DISTANCE_WAYPOINTS_THRESHOLD = 3
OBSTACLE_RADIUS = 0.5

# Planning general
TARGET_SPEED = 10.0
NUM_WAYPOINTS_AHEAD = 10
GOAL_LOCATION = np.array([0, 2])


def get_obstacle_list(obstacle_predictions, waypoints):
    if len(obstacle_predictions) == 0 or len(waypoints) == 0:
        return np.empty((0, 4))
    obstacle_list = []

    distances = pairwise_distances(waypoints, obstacle_predictions[:, :2]).min(0)
    for distance, prediction in zip(distances, obstacle_predictions):
        # Use all prediction times as potential obstacles.
        if distance < OBSTACLE_DISTANCE_WAYPOINTS_THRESHOLD:
            [x, y, _, _confidence, _label] = prediction
            obstacle_size = np.array(
                [
                    x - OBSTACLE_RADIUS,
                    y - OBSTACLE_RADIUS,
                    x + OBSTACLE_RADIUS,
                    y + OBSTACLE_RADIUS,
                ]
            )

            # Remove traffic light. TODO: Take into account traffic light.
            if _label != 9:
                obstacle_list.append(obstacle_size)

    if len(obstacle_list) == 0:
        return np.empty((0, 4))
    return np.array(obstacle_list)


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.obstacles = np.array([])
        self.position = []
        self.gps_waypoints = []
        self.waypoints = np.array([[0, 0]])
        self.obstacle_metadata = {}
        self.gps_metadata = {}
        self.metadata = {}
        self.orientation = None
        self._hyperparameters = {
            "step_size": STEP_SIZE_HYBRID_ASTAR,
            "max_iterations": MAX_ITERATIONS_HYBRID_ASTAR,
            "completion_threshold": COMPLETION_THRESHOLD,
            "angle_completion_threshold": ANGLE_COMPLETION_THRESHOLD,
            "rad_step": RAD_STEP,
            "rad_upper_range": RAD_UPPER_RANGE,
            "rad_lower_range": RAD_LOWER_RANGE,
            "obstacle_clearance": OBSTACLE_CLEARANCE_HYBRID_ASTAR,
            "lane_width": LANE_WIDTH_HYBRID_ASTAR,
            "radius": RADIUS,
            "car_length": CAR_LENGTH,
            "car_width": CAR_WIDTH,
        }

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):

        if dora_input["id"] == "position":
            self.position = np.frombuffer(dora_input["data"])

        elif dora_input["id"] == "obstacles":
            obstacles = np.frombuffer(dora_input["data"], dtype="float32").reshape(
                (-1, 5)
            )
            self.obstacles = obstacles

        (waypoints, target_speeds) = self.run(time.time())
        if len(waypoints) == 0:
            return DoraStatus.STOP
        self.waypoints = waypoints
        self.target_speeds = target_speeds

        waypoints_array = np.concatenate(
            [self.waypoints.T, self.target_speeds.reshape(1, -1)]
        )
        send_output("waypoints", waypoints_array.tobytes())
        return DoraStatus.CONTINUE

    def run(self, _ttd=None):
        """Runs the planner.
        Note:
            The planner assumes that the world is up-to-date.
        Returns:
            :py:class:`~pylot.planning.waypoints.Waypoints`: Waypoints of the
            planned trajectory.
        """

        # Remove already past waypoints
        (index, _) = closest_vertex(
            self.waypoints,
            np.array([self.position[:2]]),
        )

        self.waypoints = self.waypoints[index : index + NUM_WAYPOINTS_AHEAD]

        # Check if obstacles are on solved waypoints trajectory
        obstacle_list = get_obstacle_list(self.obstacles, self.waypoints)
        if len(obstacle_list) == 0 and self.waypoints.shape[0] > 3:
            # Do not use Hybrid A* if there are no obstacles.
            speeds = np.array([TARGET_SPEED] * len(self.waypoints))
            return self.waypoints, speeds

        # Hybrid a* does not take into account the driveable region.
        # It constructs search space as a top down, minimum bounding
        # rectangle with padding in each dimension.

        initial_conditions = self._compute_initial_conditions(obstacle_list)

        path_x, path_y, _, success = apply_hybrid_astar(
            initial_conditions, self._hyperparameters
        )

        if not success:
            print("could not find waypoints")
            print(f"initial conditions: {initial_conditions}")
            speeds = np.array([0] * len(self.waypoints))
            return [], []
        else:
            output_wps = np.array([path_x, path_y]).T
            speeds = np.array([TARGET_SPEED] * len(path_x))

            return output_wps, speeds

    def _compute_initial_conditions(self, obstacle_list):
        [x, y, _, rx, ry, rz, rw] = self.position
        orientation = R.from_quat([rx, ry, rz, rw])
        [_, _, yaw] = orientation.as_euler("xyz", degrees=False)

        start = np.array(
            [
                x,
                y,
                yaw,
            ]
        )

        [end_x, end_y] = GOAL_LOCATION
        end = np.array(
            [
                end_x,
                end_y,
                yaw,
            ]
        )

        initial_conditions = {
            "start": start,
            "end": end,
            "obs": obstacle_list,
        }
        return initial_conditions
