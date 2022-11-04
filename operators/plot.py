import time
from typing import Callable

import cv2
import numpy as np
import os
import sys

sys.path.append(os.getcwd())
from enum import Enum
from scipy.spatial.transform import Rotation as R
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

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600
DEPTH_IMAGE_WIDTH = 800
DEPTH_IMAGE_HEIGHT = 600
DEPTH_FOV = 90
SENSOR_POSITION = np.array([3, 0, 1, 0, 0, 0])
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)
writer = cv2.VideoWriter(
    "output.avi",
    cv2.VideoWriter_fourcc(*"MJPG"),
    30,
    (CAMERA_WIDTH, CAMERA_HEIGHT),
)

font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 30)
fontScale = 0.7
fontColor = (255, 0, 255)
thickness = 2
lineType = 2


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.waypoints = []
        self.obstacles = []
        self.obstacles_bbox = []
        self.obstacles_id = []
        self.lanes = []
        self.drivable_area = []
        self.last_timestamp = time.time()
        self.position = []
        self.camera_frame = []
        self.traffic_sign_bbox = []
        self.point_cloud = []
        self.control = []

    def on_input(
        self,
        dora_input: dict,
        _send_output: Callable[[str, bytes], None],
    ):

        if "waypoints" == dora_input["id"]:
            waypoints = np.frombuffer(dora_input["data"])
            waypoints = waypoints.reshape((-1, 3))
            waypoints = waypoints[:, :2]
            waypoints = np.hstack((waypoints, -0.4 + np.zeros((waypoints.shape[0], 1))))
            self.waypoints = waypoints

        elif "control" == dora_input["id"]:
            self.control = np.frombuffer(dora_input["data"])

        elif "obstacles_bbox" == dora_input["id"]:
            self.obstacles_bbox = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 6))

        elif "traffic_sign_bbox" == dora_input["id"]:
            self.traffic_sign_bbox = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 6))

        elif "obstacles_id" == dora_input["id"]:
            self.obstacles_id = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 7))

        elif "obstacles" == dora_input["id"]:
            obstacles = np.frombuffer(dora_input["data"], dtype="float32").reshape(
                (-1, 5)
            )
            self.obstacles = obstacles

        elif "lanes" == dora_input["id"]:
            lanes = np.frombuffer(dora_input["data"], dtype="int32").reshape(
                (-1, 30, 2)
            )
            self.lanes = lanes

        elif "drivable_area" == dora_input["id"]:
            drivable_area = np.frombuffer(dora_input["data"], dtype="int32").reshape(
                (1, -1, 2)
            )
            self.drivable_area = drivable_area

        elif "position" == dora_input["id"]:
            # Add sensor transform
            self.position = np.frombuffer(dora_input["data"])

        elif dora_input["id"] == "lidar_pc":
            point_cloud = np.frombuffer(dora_input["data"])
            point_cloud = np.reshape(point_cloud, (int(point_cloud.shape[0] / 5), 5))[
                :, :3
            ]

            # To camera coordinate
            # The latest coordinate space is the velodyne space.
            point_cloud = np.dot(
                point_cloud,
                np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]),
            )
            point_cloud = point_cloud[np.where(point_cloud[:, 2] > 0.1)]
            self.point_cloud = point_cloud

        elif "image" == dora_input["id"]:
            self.camera_frame = cv2.imdecode(
                np.frombuffer(
                    dora_input["data"],
                    dtype="uint8",
                ),
                -1,
            )

        if (
            "tick" != dora_input["id"]
            or isinstance(self.camera_frame, list)
            or isinstance(self.position, list)
        ):
            return DoraStatus.CONTINUE

        inv_extrinsic_matrix = np.linalg.inv(
            get_extrinsic_matrix(get_projection_matrix(self.position))
        )
        resized_image = self.camera_frame[:, :, :3]
        resized_image = np.ascontiguousarray(resized_image, dtype=np.uint8)

        ## Drawing on frame
        waypoints = location_to_camera_view(
            self.waypoints, INTRINSIC_MATRIX, inv_extrinsic_matrix
        ).T

        for waypoint in waypoints:
            cv2.circle(
                resized_image,
                (int(waypoint[0]), int(waypoint[1])),
                3,
                (
                    int(min(200 - waypoint[2] * 100, 200)),
                    int(min(waypoint[2], 255)),
                    255,
                ),
                -1,
            )

        for obstacle in self.obstacles:
            [x, y, z, _confidence, _label] = obstacle
            location = location_to_camera_view(
                np.array([[x, y, z]]),
                INTRINSIC_MATRIX,
                inv_extrinsic_matrix,
            )
            cv2.circle(
                resized_image,
                (int(location[0]), int(location[1])),
                3,
                (255, 255, 0),
                -1,
            )
            location = location_to_camera_view(
                np.array([[x, y, 0]]),
                INTRINSIC_MATRIX,
                inv_extrinsic_matrix,
            )
            cv2.circle(
                resized_image,
                (int(location[0]), int(location[1])),
                3,
                (150, 150, 0),
                -1,
            )

        locations = local_points_to_camera_view(self.point_cloud, INTRINSIC_MATRIX)

        for index, point in enumerate(locations.T):
            cv2.circle(
                resized_image,
                (int(point[0]), int(point[1])),
                3,
                (0, int(min(200 - point[2] * 200, 200)), int(min(point[2] * 10, 255))),
                -1,
            )

        for obstacle_bb in self.obstacles_bbox:
            [min_x, max_x, min_y, max_y, confidence, label] = obstacle_bb

            start = (int(min_x), int(min_y))
            end = (int(max_x), int(max_y))
            cv2.rectangle(resized_image, start, end, (0, 255, 0), 2)

            cv2.putText(
                resized_image,
                LABELS[label] + f", {confidence}%",
                (int(max_x), int(max_y)),
                font,
                0.75,
                (0, 255, 0),
                2,
                1,
            )

        for obstacle_id in self.obstacles_id:
            [
                min_x,
                max_x,
                min_y,
                max_y,
                track_id,
                confidence,
                label,
            ] = obstacle_id
            start = (int(min_x), int(min_y))
            end = (int(max_x), int(max_y))
            # cv2.rectangle(resized_image, start, end, (0, 255, 0), 2)

            cv2.putText(
                resized_image,
                f"#{track_id}",
                (int(max_x), int(max_y + 20)),
                font,
                0.75,
                (255, 140, 0),
                2,
                1,
            )

        for lane in self.lanes:
            cv2.polylines(resized_image, [lane], False, (0, 0, 255), 3)

        for contour in self.drivable_area:
            back = resized_image.copy()
            cv2.drawContours(back, [contour], 0, (0, 255, 0), -1)

            # blend with original image
            alpha = 0.25
            resized_image = cv2.addWeighted(resized_image, 1 - alpha, back, alpha, 0)
        [x, y, z, rx, ry, rz, rw] = self.position
        [_, _, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler("xyz", degrees=True)

        cv2.putText(
            resized_image,
            f"""cur: x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}""",
            (10, 30),
            font,
            fontScale,
            fontColor,
            thickness,
            lineType,
        )

        if len(self.control) != 0:
            cv2.putText(
                resized_image,
                f"""ctl: {np.degrees(self.control[0]):.2f} """,
                (10, 55),
                font,
                fontScale,
                fontColor,
                thickness,
                lineType,
            )

        writer.write(resized_image)
        cv2.imshow("image", resized_image)
        cv2.waitKey(1)
        return DoraStatus.CONTINUE
