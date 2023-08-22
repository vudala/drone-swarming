# drone-swarming
Implementing a drone swarm on PX4/AirSim simulation

### Requirements
You need all these programs:

- Python client for ROS2:
  - Install: https://github.com/ros2/rclpy

- asyncio
  - Install: pip install asyncio

- MAVSDK-Python
  - Install: pip install mavsdk

The requirements needs better documentation

If you run
```bash
pip install -r requirements.txt
```
it should be enough, but might be no so need to investigate.

### Usage
To summon NUMBER_OF_DRONES drones just run the following:
```bash
python3 maestro.py [NUMBER_OF_DRONES]
```
