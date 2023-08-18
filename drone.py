#!/usr/bin/env python3

import asyncio

import rclpy
from std_msgs.msg import ByteMultiArray

import utils
from drone_core import DroneCore
import mission


async def create(instance: int):
    name = 'drone_' + str(instance)
    drone = DroneCore(name, instance)

    sys_addr = 'udp://localhost:' + str(18570 + instance)
    print('Trying to connect to ' + sys_addr)
    await drone.connect(system_address=sys_addr)
    print('Connected to PX4')
    
    await drone.stabilize()

    return drone


async def position_refresher(drone: DroneCore, interval: float):
    """
    Keeps updating and publishing the drone position

    Parameters
    ----------
    drone: Drone
        Target drone
    interval: float
        How much time to wait between iterations
    """
    while True:
        await drone.update_position()
        drone.publish_position()
        await asyncio.sleep(interval)


# callback function of the position subscription
def subscribe_position(topic, msg):
    data = msg.data
    pos = utils.bytearray_to_obj(data)
    print('{} : {}'.format(topic, str(pos)))


# subscribe to the other drones position topic
def subscribe_to_drones_positions(drone: DroneCore, total_instances: int):
    for i in range(total_instances):
        if i != drone.instance:
            drone.subscribe_to(
                'drone_{}/position'.format(i),
                ByteMultiArray,
                subscribe_position
            )


# subscribe to all the topics
def subscribe_to_topics(drone: DroneCore, total_instances: int):
    subscribe_to_drones_positions(drone, total_instances)


# execute all the refreshing coroutines
async def refresher(drone: DroneCore):
    position_ref_coro = position_refresher(drone, 0.5)

    group = asyncio.gather(
        position_ref_coro
    )

    await group


async def execute(drone: DroneCore, total_drones: int):
    subscribe_to_topics(drone, total_drones)

    # ros2 spin on separate thread
    spin_coro = asyncio.to_thread(rclpy.spin, drone.ros2_node)

    # i wanted to run this on a separate thread but still didnt figure
    # how to do without messing with the thread scheduler
    refresher_coro = refresher(drone)

    # run mission
    mission_coro = mission.test_mission(drone, 5.5)

    # create tasks for all coroutines
    group = asyncio.gather(
        spin_coro,
        refresher_coro,
        mission_coro
    )

    # keeps waiting for them to finish (which is never)
    await group
