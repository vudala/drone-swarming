import os
import argparse

from logger import Logger

import drone

from multiprocessing import Process, Barrier


import json

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
    log_path = os.path.join(root_path(), 'log')
    log = Logger(log_path)

    log.info('Maestro initialized')
    log.info('Creating the drones')

    barrier = Barrier(parties=total_drones)

    if os.path.exists('missions.json'):
        log.info('Reading missions')
        f = open('missions.json')
        try:
            data = json.load(f)
        except Exception as e:
            log.error(e)

    procs = []
    for inst in range(total_drones):
        mission_path = data.get(f'drone_{str(inst)}', None)
        p = Process(
            target=drone.execute,
            args=[
                inst,
                total_drones,
                barrier,
                log_path,
                mission_path
            ],
            name='maestro_drone_' + str(inst)
        )
        p.start()
        procs.append(p)

        log.info('Drone {} created'.format(inst))
    
    log.info('All drones were created')
    log.info('Waiting for the drones to finish execution') 

    for p in procs:
        p.join()

    log.info('Done')


def get_args():
    parser = argparse.ArgumentParser(
        prog='python3 maestro.py',
        description='Initiates the drone swarm'
    )

    parser.add_argument(
        'total_drones', metavar='N',
        type=int, nargs=1,
        help='number of drones instances'
    )

    return parser.parse_args()


if __name__ == '__main__':
    args = get_args()
    main(args.total_drones)
