import threading
from typing import Callable

import numpy as np

from _dora_utils import DoraStatus
from carla import Client, VehicleControl, command

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
client.set_timeout(30.0)

mutex = threading.Lock()


class Operator:
    """
    Execute any `control` it r:ecieve given a `vehicle_id`
    """

    def __init__(self):
        self.vehicle_id = None

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        """Handle input.
        Args:
            dora_input["id"](str): Id of the input declared in the yaml configuration
            dora_input["data"] (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        if "position" == dora_input["id"]:
            position = np.frombuffer(dora_input["data"])
            print(f"ndt pose: {position}")
            
        return DoraStatus.CONTINUE