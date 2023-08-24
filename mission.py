from mavsdk.action import ActionError
from drone_core import DroneCore

import asyncio

from mavsdk.calibration import CalibrationError


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
        await asyncio.sleep(0.1)


async def test_mission(drone: DroneCore, target_altitude: float):
    drone.logger.info("-- Taking off")
    
    # TODO: the drone must be properly ready before you takeoff,
    # gotta find a way to wait for the commander tell its ok 
    # through MAVSDK
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(target_altitude)
    await drone.action.takeoff()


async def perform_calibration(drone: DroneCore):
    try :
        drone.logger.info("-- Starting gyroscope calibration")
        async for progress_data in drone.calibration.calibrate_gyro():
            drone.logger.info(progress_data)
        drone.logger.info("-- Gyroscope calibration finished")

    except CalibrationError as err:
        drone.logger.warning(err)


async def perform_health_checks(drone: DroneCore):
    drone.logger.info("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            drone.logger.info("-- Global position estimate OK")
            break


async def run_mission(drone: DroneCore, mission: str):
    mission_data = await drone.mission_raw.import_qgroundcontrol_mission(mission)

    await drone.mission.set_return_to_launch_after_mission(True)

    drone.logger.info("-- Uploading mission")
    await drone.mission_raw.upload_mission(mission_data.mission_items)

    await perform_health_checks(drone)

    drone.logger.info("-- Arming")
    await drone.action.arm()

    drone.logger.info("-- Starting mission")
    await drone.mission.start_mission()
