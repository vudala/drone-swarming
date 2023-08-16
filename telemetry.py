#!/usr/bin/env python3

import sys
import threading

import asyncio

import rclpy
from std_msgs.msg import ByteMultiArray

import utils
from drone import Drone


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
async def refresh_position(drone: Drone, interval: float):
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


async def run(instance: int, total_instances: int):
    # - - - - I N I T - - - -
    drone = await drone_init(instance)

    # manual semaphore, gonna change that later
    input('All set, press any button to continue')

    # - - - - S U B S C R I P T I O N - - - -
    # subscribe to positions
    subscribe_to_drones_positions(drone, instance, total_instances)

    # spin in a separate thread
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(drone.ros2_node)
    spin_thread = threading.Thread(target=executor.spin)
    spin_thread.start()


    # - - - - P U B L I S H I N G - - - -
    # updates drone position 
    refresh = asyncio.create_task(refresh_position(drone, 0.5))


    # - - - - M I S S I O N - - - -
    # ISSUE COMMANDS TO THE DRONE
    # ARM
    # TAKE OFF
    # LAND

    # dont let the main thread die
    await refresh


if __name__ == "__main__":
    # process args
    instance = int(sys.argv[1])
    total_instances = int(sys.argv[2])

    rclpy.init()
    asyncio.run(run(instance, total_instances))
    rclpy.shutdown()
