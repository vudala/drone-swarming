#!/usr/bin/env python3
# asyncio

import asyncio

# ros2 module
import rclpy
from std_msgs.msg import ByteMultiArray

# my modules
import utils
from drone_core import DroneCore
import mission



PX4_SITL_DEFAULT_PORT = 14540 

POSITION_REFRESH_INTERVAL_SEC = 0.1
DISTANCE_THRESHOLD_CM = 250

drones = []

async def create(instance: int, priority: int, logger_path: str):
    """
    Creates a drone and expects its health checks for GPS and that kind of stuff

    Parameters
    ----------
    instance: int
        Instance number of drone
    """
    name = 'drone_' + str(instance)
    drone = DroneCore(name, instance, priority, logger_path)

    sys_addr = 'udp://:' + str(PX4_SITL_DEFAULT_PORT + instance)

    drone.logger.info('Trying to connect to ' + sys_addr)
    await drone.connect(system_address=sys_addr)
    drone.logger.info('Connected to PX4')

    await drone.stabilize()

    return drone


async def position_refresher(drone: DroneCore, interval: float):
    """
    Keeps updating and publishing the drone position

    Parameters
    ----------
    - drone: DroneCore
        - Target drone
    - interval: float
        - How much time to wait between iterations (seconds)
    """
    while True:
        await drone.update_position()
        drone.publish_position()
        await asyncio.sleep(interval)


# callback function of the position subscription
def subscribe_position(drone: DroneCore, topic: str, ref: int, msg):
    data = msg.data
    pos = utils.bytearray_to_obj(data)

    drones[ref]['position'] = pos


# subscribe to the other drones position topic
def subscribe_to_drones_positions(drone: DroneCore, total_instances: int):
    for i in range(total_instances):
        if i != drone.instance:
            drone.subscribe_to(
                'drone_{}/position'.format(i),
                ByteMultiArray,
                subscribe_position,
                i
            )


# subscribe to all the topics
def subscribe_to_topics(drone: DroneCore, total_instances: int):
    subscribe_to_drones_positions(drone, total_instances)


# execute all the refreshing coroutines
async def refresher(drone: DroneCore):
    position_ref_coro = position_refresher(drone, POSITION_REFRESH_INTERVAL_SEC)

    group = asyncio.gather(
        position_ref_coro
    )

    await group


async def safechecker(drone: DroneCore, total: int):
    while True:
        for i in range(total):
            # checks if its not itself and if the i drone has already published
            # its position
            if i != drone.instance and 'position' in drones[i]:
                dist = utils.distance(drone.position, drones[i]['position'])
                if dist <= float(DISTANCE_THRESHOLD_CM):
                    # do something
                    # stall and wait for the path to be clear
                    drone.logger.warning('DRONE TOO CLOSE')
                    await drone.mission.pause_mission()
                    await drone.action.hold()
        await asyncio.sleep(0.05)


async def core(drone: DroneCore, total: int, mission_path: str):
    subscribe_to_topics(drone, total)

    drone.logger.info('Subscribed to the topics')

    coros = []

    await drone.action.set_maximum_speed(1)

    # ros2 spin on separate thread
    # TODO: this coroutine is causing thread errors
    # because asyncio is not thread safe, so we gotta fix this
    coros.append(
        asyncio.to_thread(rclpy.spin, drone.ros2_node)
    ) 

    coros.append(refresher(drone))

    if mission_path != None:
        coros.append(mission.run_mission(drone, mission_path)) 

    coros.append(safechecker(drone, total)) 

    # create tasks for all coroutines
    group = asyncio.gather(*coros)

    drone.logger.info('Running all coroutines')

    # keeps waiting for them to finish (which is never)
    await group


async def execute_core(inst, total, barrier, logger_path, mission):
    # init the drone data storage
    for i in range(total):
        if i != inst:
            drones.append(dict())
        else:
            drones.append(None)
    
    # defines the priority of the drone, for now is an arbitrary value
    priority = total - inst - 1

    dro = await create(inst, priority, logger_path)
    dro.logger.info('Drone successfully created')
    dro.logger.info('Synchronizing with the other UAVs')
    barrier.wait()
    dro.logger.info('All drones synced')
    dro.logger.info('Starting the drone')
    await core(dro, total, mission)


def execute(inst: int, total: int, barrier, logger_path, mission):
    rclpy.init()
    asyncio.run(
        execute_core(inst, total, barrier, logger_path, mission)
    )
    rclpy.shutdown()
