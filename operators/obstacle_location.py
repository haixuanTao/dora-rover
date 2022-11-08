
from dora_utils import (
    LABELS,
    DoraStatus,
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    location_to_camera_view,
    local_points_to_camera_view,
    to_world_coordinate,
)

import numpy as np
from typing import Callable

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
DEPTH_IMAGE_WIDTH = 800
DEPTH_IMAGE_HEIGHT = 480
DEPTH_FOV = 30
SENSOR_POSITION = np.array([3, 0, 1, 0, 0, 0])
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)
def get_predictions(obstacles, obstacle_with_locations):
    """Extracts obstacle predictions out of the message.
    This method is useful to build obstacle predictions when
    the operator directly receives detections instead of predictions.
    The method assumes that the obstacles are static.
    """
    predictions = []
    # Transform the obstacle into a prediction.
    for obstacle, location in zip(obstacles, obstacle_with_locations):
        obstacle = np.append(location, obstacle[-2:])
        predictions.append(obstacle)

    return predictions

class Operator:
    """
    Compute the location of obstacles.
    """

    def __init__(self):
        self.point_cloud = []
        self.obstacles = []
        self.position = []

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        if "lidar_pc" == dora_input["id"]:
            point_cloud = np.frombuffer(dora_input["data"])
            point_cloud = point_cloud.reshape((-1, 3))
            point_cloud = local_points_to_camera_view(
                point_cloud, INTRINSIC_MATRIX
            )

            if len(point_cloud) != 0:
                self.point_cloud = point_cloud
        elif "position" == dora_input["id"]:
            # Add sensor transform
            self.position = np.frombuffer(dora_input["data"])
        elif "obstacles_bbox" == dora_input["id"]:
            if len(self.position) == 0:
                return DoraStatus.CONTINUE

            self.obstacles_bbox = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 6))
        
            obstacles_with_location = []
            for obstacle_bb in self.obstacles_bbox:
                [min_x, max_x, min_y, max_y, confidence, label] = obstacle_bb
                z_points = point_cloud[np.where(point_cloud[:, 0] > min_x and 
                                                    point_cloud[:, 0] < max_x and
                                                    point_cloud[:, 1] > min_y and
                                                    point_cloud[:, 1] < max_y
                                                    )][:, 2]
                min_z = np.percentile(z_points, 10, "closest_observation")
                obstacles_with_location.append([(min_x + max_x) / 2, (min_y + max_y) / 2, min_z])
            
            
            projection_matrix = get_projection_matrix(self.position)
            extrinsic_matrix = get_extrinsic_matrix(projection_matrix)
            obstacles_with_location = to_world_coordinate(
                np.array(obstacles_with_location), extrinsic_matrix
            )
            predictions = get_predictions(self.obstacles_bbox, obstacles_with_location)
            predictions_bytes = np.array(predictions, dtype="float32").tobytes()

            send_output("obstacles", predictions_bytes, dora_input["metadata"])
        return DoraStatus.CONTINUE
