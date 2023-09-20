# standard
import os
import argparse
import json
from multiprocessing import Process, Barrier

# self
from logger import Logger
import drone


def root_path():
    """
    Gets the root path of the file

    Return
    ------
    - 
    """
    return os.path.dirname(os.path.abspath(__file__))


def read_config(logger: Logger, path: str):
    if os.path.exists(path):
        logger.info('Reading drones from {}'.format(path))
        f = open(path)
        return json.load(f)
    else:
        at_root_path = os.path.join(root_path(), 'config.json')
        if os.path.exists(at_root_path):
            logger.info('Reading drones from {}'.format(at_root_path))
            f = open(at_root_path)
            return json.load(f)
    return []


def main(fpath: str, airsim_g: bool):
    """
    Creates multiple drones and syncs them

    Parameters
    ----------
    - total_drones: int
        - How many drones to create
    """
    log_path = os.path.join(root_path(), 'log')
    log = Logger(log_path)

    log.info('Maestro initialized')
    log.info('Creating the drones')

    drones_config = read_config(log, fpath)

    total_drones = len(drones_config)
    barrier = Barrier(parties=total_drones)

    procs = []
    
    inst = 0
    for d_name in drones_config:
        m_path = drones_config[d_name].get('mission_path', None)

        if m_path == None:
            log.info(f'No mission for Drone {str(inst)}')
        
        p = Process(
            target=drone.execute,
            args=[
                d_name,
                inst,
                total_drones,
                barrier,
                log_path,
                m_path,
                airsim_g
            ],
            name='maestro_drone_' + str(inst)
        )
        p.start()
        procs.append(p)

        log.info('Drone {} created'.format(inst))

        inst += 1
    
    log.info('All drones were created')
    log.info('Waiting for the drones to finish execution') 

    for p in procs:
        p.join()

    log.info('Done')


def get_args():
    """
    Process the arguments, displays a help message if arguments were not given
    correctly

    Return
    ------
    - args: Object {drones_n, miss_path}
    """
    parser = argparse.ArgumentParser(
        prog='python3 maestro.py',
        description='Initiates the drone swarm'
    )

    parser.add_argument(
        '-c', '--config', dest='conf_path',
        metavar='filepath', type=str,
        default='config.json',
        help='path for .json config file'
    )

    parser.add_argument(
        '-a', '--airsim-external', dest='airsim_e',
        action='store_true',
        help='enable external graphic engine update for AirSim'
    )

    return parser.parse_args()


if __name__ == '__main__':
    args = get_args()
    main(args.conf_path, args.airsim_e)
