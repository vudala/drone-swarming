#!/usr/bin/env python3

import sys

import asyncio

import rclpy
from std_msgs.msg import ByteMultiArray

import utils
from drone import Drone
import mission


async def drone_init(instance: int):
    name = 'drone_' + str(instance)
    drone = Drone(name, instance)

    sys_addr = 'udp://localhost:' + str(18570 + instance)
    print('Trying to connect to ' + sys_addr)
    await drone.connect(system_address=sys_addr)
    print('Connected')
    
    await drone.stabilize()

    return drone


async def refresh_position(drone: Drone, interval: float):
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
def subscribe_to_drones_positions(drone: Drone, instance: int, total_instances: int):
    for i in range(total_instances):
        if i != instance:
            drone.subscribe_to(
                'drone_{}/position'.format(i),
                ByteMultiArray,
                subscribe_position
            )


# subscribe to all the topics
def subscribe_to_topics(drone: Drone, instance: int, total_instances: int):
    subscribe_to_drones_positions(drone, instance, total_instances)


# execute all the refreshing coroutines
async def refresher(drone: Drone):
    position_ref_coro = refresh_position(drone, 0.5)

    group = asyncio.gather(
        position_ref_coro
    )

    await group


async def run(instance: int, total_instances: int):
    drone = await drone_init(instance)

    # manual semaphore, gonna change that later
    input('All set, press any button to continue')

    subscribe_to_topics(drone, instance, total_instances)

    # ros2 spin on separate thread
    spin_coro = asyncio.to_thread(rclpy.spin, drone.ros2_node)

    # i wanted to run this on a separate thread but still didnt figure how to do without messing with the thread scheduler
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


if __name__ == "__main__":
    # process args
    instance = int(sys.argv[1])
    total_instances = int(sys.argv[2])

    rclpy.init()
    asyncio.run(run(instance, total_instances))
    rclpy.shutdown()
