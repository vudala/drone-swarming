from rclpy.node import Node
from mavsdk import System

from std_msgs.msg import ByteMultiArray, ByteMultiArray

from functools import partial

from collections.abc import Callable

import utils

from logger import Logger


MAVSDK_SERVER_DEFAULT_PORT = 50051


class DroneCore(System):
    """
    Wraps the ROS2 and MAVSDK functionalities into an abstraction of a UAV
    """
    def __init__(self, name: str, instance: int, priority: int, logger_path: str):
        """
        Inits the attributes, mavsdk_server and ros2 node and publishers
        """
        super().__init__(port=(MAVSDK_SERVER_DEFAULT_PORT + instance))

        self.name = name

        self.priority = priority

        self.ros2_node = Node(name)
        self.instance = instance

        self.position = None
        self.position_publisher = self.create_publisher('/position', ByteMultiArray)

        self.subscribed = set()

        self.logger = Logger(
            logger_path,
            'drone_{}'.format(instance)
        )

    
    async def stabilize(self):
        """
        Wait for the sensors to stabilize
        """
        self.logger.info("Waiting for drone to connect...")
        async for state in self.core.connection_state():
            if state.is_connected:
                self.logger.info(f"-- Connected to drone!")
                break

        self.logger.info("Waiting for drone to have a global position estimate...")
        async for health in self.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.logger.info("-- Global position estimate OK")
                break


    async def update_position(self):
        """
        Update its own position
        """
        async for pos in self.telemetry.position():
            self.position = pos
            return
        

    def create_publisher(self, topic: str, data_type: any):
        """
        Creates a publisher for some topic, with QoS 1 to allow only the latest info to be transmited

        Parameters
        ----------
        - topic: str
            Name of the ROS2 topic you want tocreate_publisher
        """
        node = self.ros2_node
        return node.create_publisher(data_type, self.name + topic, 1)


    def publish_position(self):
        """
        Publish its own position at the position topic
        """
        msg = ByteMultiArray()
        msg.data = utils.obj_to_bytearray(self.position)
        self.position_publisher.publish(msg)


    def subscribe_to(self, topic: str, data_type: any, callback: Callable[[any, str, int, any], None], ref: int):
        """
        Subscribes to a topic

        Parameters
        ----------
        - topic: str
            - Name of the topic to subscribe to
        - data_type: any
            - ROS2 data type to be used at the callback
        - callback: function(drone, topic, ref, msg)
            - A callback function with the first param being the topic and the second the message of the callback param
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
