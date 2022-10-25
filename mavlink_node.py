#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from dora import Node
import numpy as np


async def run():
    drone = System()
    print("Waiting for drone...")
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state}")
            break

    await drone.action.arm()

    for _ in range(5):
        input_id, value, metadata = node.next()
        [vx, vy, vz, yaw] = np.frombuffer(value)
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(vx, vy, vz, yaw))
        await asyncio.sleep(0.5)
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0.0, 0.0))
        await asyncio.sleep(0.5)

    print("-- Landing")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
