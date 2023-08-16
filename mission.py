from mavsdk.action import ActionError
from drone import Drone

import asyncio

async def assert_action(action):
    while True:
        try:
            await action 
            return
        except ActionError as err:
            print(err)
        except Exception as err:
            print(err)
        asyncio.sleep(0.1)


async def test_mission(drone: Drone, target_altitude: float):
    print("-- Taking off")
    #await assert_action(drone.action.arm())
    #await assert_action(drone.action.set_takeoff_altitude(target_altitude))
    #await drone.action.takeoff()