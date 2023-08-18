from rclpy.node import Node
from mavsdk import System

from std_msgs.msg import ByteMultiArray, ByteMultiArray

from functools import partial

from collections.abc import Callable

import utils


MAVSDK_SERVER_DEFAULT_PORT = 50051

class DroneCore(System):
    """
    Wraps the ROS2 and MAVSDK functionalities into an abstraction of a UAV
    """
    def __init__(self, name: str, instance: int):
        """
        Inits the attributes, mavsdk_server and ros2 node and publishers
        """
        super().__init__(port=(MAVSDK_SERVER_DEFAULT_PORT + instance))

        self.name = name

        self.ros2_node = Node(name)
        self.instance = instance

        self.position = None
        self.position_publisher = self.create_publisher('/position', ByteMultiArray)

        self.subscribed = set()

    
    async def stabilize(self):
        """
        Wait for the sensors to stabilize
        """
        print("Waiting for drone to connect...")
        async for state in self.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

        print("Waiting for drone to have a global position estimate...")
        async for health in self.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
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
        Creates a publisher for some topic, with QoS 1 to allow only the latest info

        Parameters
        ----------
        topic: str
            Name of the ROS2 topic you want to publish, if the topic doesn't exist, creates one with such name
        data_type: any
            Any data type supported by ROS2
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


    def subscribe_to(self, topic: str, data_type: any, callback: Callable[[str, any], None]):
        """
        Subscribes to a topic

        Parameters
        ----------
        topic: str
            Name of the topic to subscribe to
        data_type: any
            ROS2 data type to be used at the callback
        callback: function(topic, msg)
            A callback function with the first param being the topic and the second the message of the callback param
        """
        sub = self.ros2_node.create_subscription(
            data_type,
            topic,
            partial(callback, topic),
            1
        )
        self.subscribed.add(sub)
