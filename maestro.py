import sys
import asyncio
import rclpy

import drone

from multiprocessing import Process, Barrier


async def core(inst, total, barrier):
    drone = await drone.create(inst)
    barrier.wait()
    await drone.execute(drone, total)


def run(instance: int, total_drones: int, barrier: Barrier):
    rclpy.init()
    asyncio.run(core(instance, total_drones, barrier))
    rclpy.shutdown()


def main(total_drones: int):
    barrier = Barrier(parties=total_drones)

    procs = []
    for i in range(total_drones):
        p = Process(
            target=run,
            args=[i, total_drones, barrier],
            name='maestro_drone_' + str(i)
        )
        p.start()
        procs.append(p)
    
    for p in procs:
        p.join()


if __name__ == '__main__':
    total_drones = int(sys.argv[1])
    main(total_drones)