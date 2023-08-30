#!/usr/bin/env python3
# asyncio

import asyncio
from multiprocessing.synchronize import Barrier

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
    - instance: int
        - Instance number of drone
    
    Return
    ------
    - drone: DroneCore
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


def subscribe_to_drones_positions(drone: DroneCore, total_instances: int):
    """
    Subscribes to the position of the other drones

    Parameters
    ---------
    - drone: DroneCore
        - The drone that will be subscribing
    - total_instances: int
        - Total number of drones in the swarm
    """
    for i in range(total_instances):
        if i != drone.instance:
            drone.subscribe_to(
                'drone_{}/position'.format(i),
                ByteMultiArray,
                subscribe_position,
                i
            )


def subscribe_to_topics(drone: DroneCore, total_instances: int):
    """
    Run all of the subscription routines

    - drone: DroneCore
        - Target drone
    - total_instances: int
        - Total drones in the swarm
    """
    subscribe_to_drones_positions(drone, total_instances)


async def refresher(drone: DroneCore):
    """
    Creates and executes all of the refresher coroutines

    Parameters
    - drone: DroneCore
        - Target drone of the refreshing tasks
    """
    position_ref_coro = position_refresher(drone, POSITION_REFRESH_INTERVAL_SEC)

    group = asyncio.gather(
        position_ref_coro
    )

    await group


async def safechecker(drone: DroneCore, total: int):
    """
    Checks if its too clone to any other drone
    
    Paramenters
    -----------
    - drone: DroneCore
        - The drone that will be checking
    - total: int
        - Total number of drones in the swarm
    """
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


async def start_coroutines(
        drone: DroneCore, total: int,
        mission_path: str = None
    ):
    """
    Setup and kickstart all the coroutines of the drone

    Parameters
    - drone: DroneCore
        - Target drone
    - total: int
        - Total number of drones in the swarm
    - mission_path: str
        - Path to .plan missions file
    """
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


async def execute_core(
        inst: int, total: int,
        barrier: Barrier,
        logger_path: str, mission: str
    ):
    # init the drones data storage
    for i in range(total):
        if i != inst:
            drones.append(dict())
        else:
            drones.append(None)
    
    # defines the priority of the drone, for now is an arbitrary value
    priority = inst

    dro = await create(inst, priority, logger_path)
    dro.logger.info('Drone successfully created')
    
    dro.logger.info('Synchronizing with the other UAVs')
    barrier.wait()
    dro.logger.info('Synchronized')

    dro.logger.info('Subscribing to the other drones topics')
    subscribe_to_topics(dro, total)
    dro.logger.info('Subscribed to the topics')

    dro.logger.info('All drones synced')
    dro.logger.info('Starting the coroutines')
    await start_coroutines(dro, total, mission)


def execute(
        inst: int, total: int,
        barrier: Barrier,
        logger_path: str, mission_path: str):
    """
    Executes all the tasks of a drone

    Parameters
    ----------
    - inst: int
        - Number of the drone's instance
    - total: int
        - How many drones are being simulated
    - barrier: Barrier
        - A Barrier used to synchronize all the drones, must have the parties
        number the same as the total param
    - logger_path: str
        - Root dir of the logger system
    - mission_path: str
        - Path to .plan missions file
    """
    rclpy.init()
    asyncio.run(
        execute_core(inst, total, barrier, logger_path, mission_path)
    )
    rclpy.shutdown()
