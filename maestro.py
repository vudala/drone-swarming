import os
import sys

from logger import Logger

import drone

from multiprocessing import Process, Barrier


def root_path():
    return os.path.dirname(os.path.abspath(__file__))


def main(total_drones: int):
    """
    Creates multiple drones and syncs them

    Parameters
    ----------
    total_drones: int
        How many drones to create
    """
    path = os.path.join(root_path(), 'log')
    log = Logger(path)

    log.info('Maestro initialized')
    log.info('Creating the drones')

    barrier = Barrier(parties=total_drones)

    procs = []
    for inst in range(total_drones):
        p = Process(
            target=drone.execute,
            args=[
                inst,
                total_drones,
                barrier,
                path
            ],
            name='maestro_drone_' + str(inst)
        )
        p.start()
        procs.append(p)

        log.info('Drone {} created'.format(inst))
    
    log.info('All drones were created')
    log.info('Waiting for the to finish execution') 

    for p in procs:
        p.join()

    log.info('Done')


if __name__ == '__main__':
    total_drones = int(sys.argv[1])
    main(total_drones)
