import sys
import asyncio
import rclpy

import telemetry

from multiprocessing import Process

import concurrent.futures

# create a bunch of drones
# wait for them to yield
# resume their execution


async def init_drones(total_drones: int):
    creators = []
    for i in range(total_drones):
        creators.append(telemetry.create_drone(i))

    # wait for all the drones to be properly initalized
    group = asyncio.gather(*creators)
    await group

    return group.result()


async def main(total_drones: int):
    drones = await init_drones(total_drones)

    executor = rclpy.executors.SingleThreadedExecutor()

    tasks = []
    for d in drones:
        executor.add_node(d.ros2_node)
        tsk = telemetry.execute(d, total_drones)
        tasks.append(tsk)
        
    spinner = asyncio.to_thread(executor.spin)
    group = asyncio.gather(*tasks, spinner)
    await group
    

if __name__ == '__main__':
    total_drones = int(sys.argv[1])
    rclpy.init()
    asyncio.run(main(total_drones))
    rclpy.shutdown()
