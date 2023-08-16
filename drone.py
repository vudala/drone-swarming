import asyncio

from rclpy.node import Node
from mavsdk import System

from std_msgs.msg import ByteMultiArray, ByteMultiArray

import pickle

from functools import partial

from collections.abc import Callable

import utils


MAVSDK_SERVER_DEFAULT_PORT = 50051

class Drone(System):
    def __init__(self, name: str, instance: int):
        super().__init__(port=(MAVSDK_SERVER_DEFAULT_PORT + instance))

        self.name = name

        self.ros2_node = Node(name)
        self.instance = instance

        self.position = None
        self.position_publisher = self.create_publisher('/position', ByteMultiArray)

        self.subscribed = set()

    
    # wait for the sensors to stabilize
    async def stabilize(self):
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

    # update its own position
    async def update_position(self):
        async for pos in self.telemetry.position():
            self.position = pos
            return
        

    # creates a publisher for some topic
    def create_publisher(self, topic: str, data_type: any):
        node = self.ros2_node
        print(self.name + topic + ' publisher created')
        return node.create_publisher(data_type, self.name + topic, 1)


    # publish its own position at the topic
    def publish_position(self):
        msg = ByteMultiArray()
        msg.data = utils.obj_to_bytearray(self.position)
        self.position_publisher.publish(msg)


    # subscribes to some topic
    def subscribe_to(self, topic: str, data_type: any, callback: Callable[[str, any], None]):
        sub = self.ros2_node.create_subscription(
            data_type,
            topic,
            partial(callback, topic),
            1
        )
        self.subscribed.add(sub)
