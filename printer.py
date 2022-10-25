import threading
from typing import Callable
from enum import Enum
import numpy as np


mutex = threading.Lock()
class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1

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