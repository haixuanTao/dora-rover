from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper import (
    apply_hybrid_astar,
)
import numpy as np

STEP_SIZE_HYBRID_ASTAR = 1
MAX_ITERATIONS_HYBRID_ASTAR = 2000
COMPLETION_THRESHOLD = 1
ANGLE_COMPLETION_THRESHOLD = 4
RAD_STEP = 0.1
RAD_UPPER_RANGE = 4
RAD_LOWER_RANGE = 4
OBSTACLE_CLEARANCE_HYBRID_ASTAR = 0.2
LANE_WIDTH_HYBRID_ASTAR = 10.0
RADIUS = 0.3
CAR_LENGTH = 0.6
CAR_WIDTH = 0.3

OBSTACLE_DISTANCE_WAYPOINTS_THRESHOLD = 3
OBSTACLE_RADIUS = 0.5

# Planning general
TARGET_SPEED = 10.0
NUM_WAYPOINTS_AHEAD = 10
GOAL_LOCATION = np.array([0, 2])

hyperparameters = {
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

initial_conditions = {
    "start": np.array([0.0, 0, 0]),
    "end": np.array([5.0, 5, 0]),
    "obs": np.empty((0, 4)),
}

path_x, path_y, _, success = apply_hybrid_astar(initial_conditions, hyperparameters)

print(success)
print(path_x)
print(path_y)
