from mavsdk.action import ActionError
from drone import Drone

import asyncio


# this need to be reformulated, it has problems with it
async def assert_action(action):
    """
    The main objective of this function is to assure that
    the given action is exectued, because an action can throw
    an exception during its execution
    
    action: Coroutine
        This is a coroutine obtained from invoking an action from drone.action, such as drone.action.arm()
    """
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