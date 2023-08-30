import os
import argparse
import json
from multiprocessing import Process, Barrier


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
        

def read_missions(logger: Logger, path: str):
    """
    Parse a .json missions file

    Parameters
    ----------
    - logger: Logger
        - Logger object used to log info
    - path: str
        - Path to the .json missions file

    Return
    ------
    - If there's no configuration file available returns None
    - Else returns the processed json data
    """
    if os.path.exists(path):
        logger.info('Reading missions from {}'.format(path))
        f = open(path)
        return json.load(f)
    else:
        at_root_path = os.path.join(root_path(), 'missions.json')
        if os.path.exists(at_root_path):
            logger.info('Reading missions from {}'.format(at_root_path))
            f = open(at_root_path)
            return json.load(f)
    
    logger.warning('No missions file was given')
    return None


def main(total_drones: int, missions_path: str):
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

    barrier = Barrier(parties=total_drones)

    m_data = read_missions(log, missions_path)

    procs = []
    for inst in range(total_drones):
        m_path = None

        if m_data:
            m_path = m_data.get(f'drone_{str(inst)}', None)

        if m_path == None:
            log.info(f'No mission for Drone {str(inst)}')
        
        p = Process(
            target=drone.execute,
            args=[
                inst,
                total_drones,
                barrier,
                log_path,
                m_path
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
        'drones_n', metavar='drones_n',
        type=int,
        help='number of drone instances'
    )

    parser.add_argument(
        '--missions', dest='miss_path',
        metavar='filepath', type=str,
        default='missions.json',
        help='path for .json missions file'
    )

    return parser.parse_args()


if __name__ == '__main__':
    args = get_args()
    main(args.drones_n, args.miss_path)
