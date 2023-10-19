# standard
import time
from functools import partial
from typing import Callable

# 3rd party
import asyncio
from rclpy.node import Node
from mavsdk import System
from std_msgs.msg import ByteMultiArray, ByteMultiArray

# self
import utils
from logger import Logger


MAVSDK_SERVER_DEFAULT_PORT = 50051


class DroneCore(System):
    """
    Wraps the ROS2 and MAVSDK functionalities into an abstraction of a UAV
    """
    def __init__(
            self, name: str,
            instance: int, priority: int,
            logger_path: str
        ):
        """
        Inits the attributes, mavsdk_server, ros2 node and publishers
        """
        super().__init__(port=(MAVSDK_SERVER_DEFAULT_PORT + instance))

        self.name = name

        self.priority = priority

        self.node_name = 'drone_{}'.format(instance)
        self.ros2_node = Node(self.node_name)
        self.instance = instance

        self.subscribed = set()

        self.logger = Logger(
            logger_path,
            'drone_{}'.format(instance)
        )

        self.position = None
        self.position_publisher = self.create_publisher(
            '/position',
            ByteMultiArray
        )
        self.relative_alt_m = None

        self.velocity_ned = None
        self.ground_speed_ms = None

        self.fixedw = None
        self.airspeed_ms = None
        self.throttle_pct = None
        self.climb_rate_ms = None

        self.odometry = None
        self.roll_rads = None
        self.pitch_rads = None
        self.yaw_rads = None

        self.battery_pct = None
        

    async def stabilize(self):
        """
        Wait for the sensors to stabilize
        """
        self.logger.info("Waiting for drone to connect...")
        async for state in self.core.connection_state():
            if state.is_connected:
                self.logger.info(f"-- Connected to drone!")
                break

        self.logger.info(
            "Waiting for drone to have a global position estimate..."
        )
        async for health in self.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.logger.info("-- Global position estimate OK")
                break
        

    def create_publisher(self, topic: str, data_type: any):
        """
        Creates a publisher for some topic, with QoS 1 to allow only the latest
        info to be transmited

        Parameters
        ----------
        - topic: str
            - Name of the ROS2 topic you want tocreate_publisher
        - data_type: any
            - Data type of the topic
        """
        node = self.ros2_node
        return node.create_publisher(data_type, self.node_name + topic, 1)


    def publish_position(self):
        """
        Publish its own position at the position topic
        """
        if self.position != None:
            msg = ByteMultiArray()
            msg.data = utils.obj_to_bytearray(self.position)
            self.position_publisher.publish(msg)


    def subscribe_to(
            self,
            topic: str, data_type: any,
            callback: Callable[[any, str, int, any], None],
            ref: int
        ):
        """
        Subscribes to a topic

        Parameters
        ----------
        - topic: str
            - Name of the topic to subscribe to
        - data_type: any
            - ROS2 data type to be used at the callback
        - callback: function(drone, topic, ref, msg)
            - A callback function with the first param being the topic and the
            second the message of the callback param
        - ref: int
            - Instance that it refers to
        """
        sub = self.ros2_node.create_subscription(
            data_type,
            topic,
            partial(callback, self, topic, ref),
            1
        )
        self.subscribed.add(sub)

        
    async def position_refresher(self, delay):
        """
        Keeps updating and publishing the drone position

        Parameters
        ----------
        - delay: float
            - Delay in seconds between iterations
        """
        await self.telemetry.set_rate_position(20)
        async for pos in self.telemetry.position():
            self.position = pos
            self.relative_alt_m = pos.relative_altitude_m
            self.publish_position()
            await asyncio.sleep(delay)


    async def velocity_refresher(self, delay):
        """
        Keeps updating and publishing the drones NED velocity

        Parameters
        ----------
        - delay: float
            - Delay in seconds between iterations
        """
       # await self.telemetry.set_rate_velocity_ned(2)
        async for v in self.telemetry.velocity_ned():
            self.velocity_ned = v
            await asyncio.sleep(delay)


    async def gnd_speed_refresher(self, delay):
        """
        Keeps updating drones ground speed

        Parameters
        ----------
        - delay: float
            - Delay in seconds between iterations
        """
        while True:
            await asyncio.sleep(delay)
            vel = self.velocity_ned
            if vel == None:
                continue
            self.ground_speed_ms = utils.ground_speed_ms(vel)


    async def thrt_crate_refresher(self, delay):
        """
        Keeps updating drones throttle and climb rate

        Parameters
        ----------
        - delay: float
            - Delay in seconds between iterations
        """
        await self.telemetry.set_rate_fixedwing_metrics(20)
        async for met in self.telemetry.fixedwing_metrics():
            self.fixedw = met
            self.airspeed_ms = met.airspeed_m_s
            self.throttle_pct = met.throttle_percentage
            self.climb_rate_ms = met.climb_rate_m_s
            await asyncio.sleep(delay)


    async def odometry_refresher(self, delay):
        """
        Keeps updating the odometry data of the drone

        Parameters
        ----------
        - delay: float
            - Delay in seconds between iterations
        """
        await self.telemetry.set_rate_odometry(20)
        async for od in self.telemetry.odometry():
            self.odometry = od
            self.roll_rads = od.angular_velocity_body.roll_rad_s
            self.pitch_rads = od.angular_velocity_body.pitch_rad_s
            self.yaw_rads = od.angular_velocity_body.yaw_rad_s
            await asyncio.sleep(delay)


    async def battery_refresher(self, delay):
        """
        Keeps updating the battery percentage of the drone

        Parameters
        ----------
        - delay: float
            - Delay in seconds between iterations
        """

        async def assert_init():  
            gnd = self.ground_speed_ms
            fwm = self.fixedw
            odo = self.odometry
            pos = self.position

            while None in [gnd, fwm, odo, pos]:
                await asyncio.sleep(0.1)
                gnd = self.ground_speed_ms
                fwm = self.fixedw
                odo = self.odometry
                pos = self.position

        await assert_init()

        battery_capacity_mah = 10000
        average_voltage = 22.5453
        battery_capacity_wh = (battery_capacity_mah * average_voltage) / 1000

        prev_time = time.time()
        total_energy_consumed = 0.0

        while True:
            gnd = self.ground_speed_ms
            air = self.airspeed_ms
            roll = self.roll_rads
            pitch = self.pitch_rads
            yaw = self.yaw_rads
            alt = self.relative_alt_m
            thr = self.throttle_pct

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
            
            self.battery_pct = remaining_percentage

            await asyncio.sleep(delay)
