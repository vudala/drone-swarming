# standard
from multiprocessing.synchronize import Barrier
import time

# 3rd party
import asyncio
import rclpy
import airsim
from std_msgs.msg import ByteMultiArray

# self
import utils
from drone_core import DroneCore
import mission


PX4_SITL_DEFAULT_PORT = 14540

# delay in seconds
POSITION_REFRESH_DELAY = 0.1
VELOCITY_REFRESH_DELAY = 0.1
GND_SPEED_REFRESH_DELAY = 0.1
THRT_CRATE_REFRESH_DELAY = 0.1
ODOMETRY_REFRESH_DELAY = 0.1

DISTANCE_THRESHOLD_CM = 200

drones = []

async def create(name: str, instance: int, priority: int, logger_path: str):
    """
    Creates a drone and expects its health checks for GPS and that kind of stuff

    Parameters
    ----------
    - name: str
        - Name of the drone
    - instance: int
        - Instance number of drone
    
    Return
    ------
    - drone: DroneCore
    """
    drone = DroneCore(name, instance, priority, logger_path)

    sys_addr = 'udp://:' + str(PX4_SITL_DEFAULT_PORT + instance)

    drone.logger.info('Trying to connect to ' + sys_addr)
    await drone.connect(system_address=sys_addr)
    drone.logger.info('Connected to PX4')

    await drone.stabilize()

    return drone


# callback function of the position subscription
def subscribe_position(drone: DroneCore, topic: str, ref: int, msg):
    data = msg.data
    pos = utils.bytearray_to_obj(data)
    
    drones[ref]['position'] = pos


def subscribe_to_topics(drone: DroneCore, total_instances: int):
    """
    Subscribes to the topics of the other drones

    Parameters
    ---------
    - drone: DroneCore
        - The drone that will be subscribing
    - total_instances: int
        - Total number of drones in the swarm
    """
    for i in range(total_instances):
        if i != drone.instance:
            # Position
            drone.subscribe_to(
                'drone_{}/position'.format(i),
                ByteMultiArray,
                subscribe_position,
                i
            )


async def airsim_pos_updater(drone: DroneCore):
    """
    Connects to AirSim API and starts to set the position of the corresponding
    aircraft according to the PX4 SITL provided position and orientation

    The vehicle name must be the same for AirSim and maestro
    """
    client = airsim.MultirotorClient()
    client.confirmConnection()
    while True:
        odo = drone.odometry
        if odo != None:
            pos = odo.position_body
            quat = odo.q

            pose = {
                'orientation': {
                    'w_val': quat.w,
                    'x_val': quat.x,
                    'y_val': quat.y,
                    'z_val': quat.z,
                },
                'position': {   
                    'x_val': pos.x_m, 
                    'y_val': pos.y_m,  
                    'z_val': pos.z_m
                }
            }

            client.simSetVehiclePose(
                pose,
                ignore_collision=True,
                vehicle_name=drone.name
            )
        await asyncio.sleep(0.01)


async def refresher(drone: DroneCore):
    """
    Creates and executes all of the refresher coroutines

    Parameters
    - drone: DroneCore
        - Target drone of the refreshing tasks
    """
    position_ref_coro = drone.position_refresher(POSITION_REFRESH_DELAY)
    velocity_ref_coro = drone.velocity_refresher(VELOCITY_REFRESH_DELAY)
    gnd_speed_ref_coro = drone.gnd_speed_refresher(GND_SPEED_REFRESH_DELAY)
    fixedw_ref_coro = drone.thrt_crate_refresher(THRT_CRATE_REFRESH_DELAY)
    odometry_ref_coro = drone.odometry_refresher(ODOMETRY_REFRESH_DELAY)

    group = asyncio.gather(
        position_ref_coro,
        velocity_ref_coro,
        gnd_speed_ref_coro,
        fixedw_ref_coro,
        odometry_ref_coro,
    )

    await group


async def safechecker(drone: DroneCore, total: int):
    """
    Checks if its too clone to any other drone
    
    Paramenters
    -----------
    - drone: DroneCore
        - The drone that will be checkingbattery capacity mah
    """
    while True:
        for i in range(total):
            # checks if its not itself and if the i drone has already published
            # its position
            if i != drone.instance and 'position' in drones[i]:
                dist = utils.distance_cm(drone.position, drones[i]['position'])
                if dist <= float(DISTANCE_THRESHOLD_CM):
                    # do something
                    # stall and wait for the path to be clear
                    drone.logger.warning('DRONE TOO CLOSE')
                    await drone.mission.pause_mission()
                    await drone.action.hold()
        await asyncio.sleep(0.05)


async def assert_init(drone: DroneCore):
    """
    Asserts that all of the sensor data has been fetched
    - drone: DroneCore
        - Target drone
    """
    gnd = drone.ground_speed_ms
    fwm = drone.fixedw
    odo = drone.odometry
    pos = drone.position

    while None in [gnd, fwm, odo, pos]:
        await asyncio.sleep(0.1)
        gnd = drone.ground_speed_ms
        fwm = drone.fixedw
        odo = drone.odometry
        pos = drone.position


async def battery_logger(drone: DroneCore):
    await assert_init(drone)

    battery_capacity_mah = 10000
    average_voltage = 22.5453
    battery_capacity_wh = (battery_capacity_mah * average_voltage) / 1000

    prev_time = time.time()
    total_energy_consumed = 0.0

    while True:
        gnd = drone.ground_speed_ms
        air = drone.airspeed_ms
        roll = drone.roll_rads
        pitch = drone.pitch_rads
        yaw = drone.yaw_rads
        alt = drone.relative_alt_m
        thr = drone.throttle_pct

        predicted_power = (
            571.52 + (3.8 * gnd) - (14.85 * air) + (0.012 * roll)
            + (3.44 * pitch) - (0.058 * yaw) - (1.46 * alt) + (12.0 * thr)
        )
        t = time.time()
        time_interval = t - prev_time
        prev_time = t

        total_energy_consumed += (predicted_power * time_interval) / 3600
        remaining_energy = battery_capacity_wh - total_energy_consumed
        remaining_percentage  = (remaining_energy / battery_capacity_wh)

        msg = "pct: {} te_cons: {}".format(
            remaining_percentage,
            total_energy_consumed
        )
        drone.logger.info(msg)

        # msg = "gnd: {} air: {} roll: {} pitch: {} yaw: {} alt: {} thr: {}".format(
        #     gnd, air, roll, pitch, yaw, alt, thr
        # )
        # drone.logger.error(msg)

        await asyncio.sleep(0.1)


# async def battery_logger(drone: DroneCore):
#     async for bat in drone.telemetry.battery():
#         bt = time.time()
#         rp = bat.remaining_percent
#         gnd = drone.ground_speed_ms
#         air = drone.airspeed_ms
#         alt = drone.position.absolute_altitude_m
#         yaw = drone.yaw_rads
#         pitch = drone.pitch_rads
#         roll = drone.pitch_rads

#         msg = "{}, {}, {}, {}, {}, {}, {}, {}".format(
#             bt, rp, gnd, air, alt ,yaw, pitch, roll
#         )
#         drone.logger.info(msg)


async def start_coroutines(
        drone: DroneCore, total: int,
        mission_path: str = None,
        airsim_e: bool = False
    ):
    """
    Setup and kickstart all the coroutines of the drone

    Parameters
    - drone: DroneCore
        - Target drone
    - total: int
        - Total number of drones in the swarm
    - mission_path: str
        - Path to .plan missions file
    """
    coros = []

    # ros2 spin on separate thread
    coros.append(
        asyncio.to_thread(rclpy.spin, drone.ros2_node)
    ) 

    coros.append(refresher(drone))

    if mission_path != None:
        coros.append(mission.run_mission(drone, mission_path)) 

    coros.append(safechecker(drone, total))

    if airsim_e:
        coros.append(airsim_pos_updater(drone))

    coros.append(battery_logger(drone))

    # create tasks for all coroutines
    group = asyncio.gather(*coros)

    drone.logger.info('Running all coroutines')

    await group


async def execute_core(
        name: str, inst: int, total: int,
        barrier: Barrier,
        logger_path: str, mission: str, airsim: bool
    ):
    """
    Wraps the functionalities
    """
    # init the drones data storage
    for i in range(total):
        if i != inst:
            drones.append(dict())
        else:
            drones.append(None)
    
    # defines the priority of the drone, for now is an arbitrary value
    priority = inst

    dro = await create(name, inst, priority, logger_path)
    dro.logger.info('Drone successfully created')
    
    dro.logger.info('Synchronizing with the other UAVs')
    barrier.wait()
    dro.logger.info('Synchronized')

    dro.logger.info('Subscribing to the other drones topics')
    subscribe_to_topics(dro, total)
    dro.logger.info('Subscribed to the topics')

    dro.logger.info('All drones synced')
    dro.logger.info('Starting the coroutines')
    await start_coroutines(dro, total, mission, airsim)


def execute(
        name: str, inst: int, total: int,
        barrier: Barrier,
        logger_path: str, mission_path: str, airsim_e: bool):
    """
    Executes all the tasks of a drone

    Parameters
    ----------
    - name: str
        - Name of the drone
    - inst: int
        - Number of the drone's instance
    - total: int
        - How many drones are being simulated
    - barrier: Barrier
        - A Barrier used to synchronize all the drones, must have the parties
        number the same as the total param
    - logger_path: str
        - Root dir of the logger system
    - mission_path: str
        - Path to .plan missions file
    """
    rclpy.init()
    asyncio.run(
        execute_core(
            name,
            inst,
            total,
            barrier,
            logger_path,
            mission_path,
            airsim_e
        )
    )
    rclpy.shutdown()
