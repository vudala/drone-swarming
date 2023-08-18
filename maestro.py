import sys
import asyncio
import rclpy

import telemetry

from multiprocessing import Process

import concurrent.futures

# create a bunch of drones
# wait for them to yield
# resume their execution


async def run_drone(instance: int, total_drones: int):
    drone = await telemetry.create_drone(instance)
    await telemetry.execute(drone, total_drones)


def run(instance: int, total_drones: int):
    rclpy.init()
    asyncio.run(run_drone(instance, total_drones))
    rclpy.shutdown()


def main(total_drones: int):
    procs = []
    for i in range(total_drones):
        p = Process(
            target=run,
            args=[i, total_drones]
        )
        p.start()
        procs.append(p)

    for p in procs:
        p.join()


if __name__ == '__main__':
    total_drones = int(sys.argv[1])
    main(total_drones)