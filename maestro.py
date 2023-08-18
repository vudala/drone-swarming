import sys
import asyncio
import rclpy

import drone

from multiprocessing import Process, Barrier


async def core(inst, total, barrier):
    dro = await drone.create(inst)
    barrier.wait()
    await drone.execute(dro, total)


def run(instance: int, total_drones: int, barrier: Barrier):
    rclpy.init()
    asyncio.run(core(instance, total_drones, barrier))
    rclpy.shutdown()


def main(total_drones: int):
    """
    Creates multiple drones and syncs them

    Parameters
    ----------
    total_drones: int
        How many drones to create
    """
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