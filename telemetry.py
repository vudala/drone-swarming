#!/usr/bin/env python3

import sys
import pickle
import threading

import asyncio

from drone import Drone

import rclpy

from std_msgs.msg import ByteMultiArray

import mission

import utils


async def drone_init(instance: int):
    name = 'drone_' + str(instance)
    drone = Drone(name, instance)

    sys_addr = 'udp://localhost:' + str(18570 + instance)
    print('Trying to connect to ' + sys_addr)
    await drone.connect(system_address=sys_addr)
    print('Connected')
    
    await drone.stabilize()

    return drone


# keeps updating and publishing its own position
async def refresh_position(drone: Drone):
    while True:
        await drone.update_position()
        drone.publish_position()
        await asyncio.sleep(0.5)


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


async def run():
    # process args
    instance = int(sys.argv[1])
    target_altitude = float(sys.argv[2])
    total_instances = int(sys.argv[3])

    # init
    drone = await drone_init(instance)

    # manual semaphore, gonna change that later
    input('Press any button to continue')

    subscribe_to_drones_positions(drone, instance, total_instances)

    # spin in a separate thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(drone.ros2_node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()
    
    # keeps updating the drone position 
    task = asyncio.create_task(refresh_position(drone))

    await mission.test_mission(drone, target_altitude)

    await task


if __name__ == "__main__":
    rclpy.init()
    asyncio.run(run())
    rclpy.shutdown()
