# Maestro
Implementing a drone swarm controller on PX4 SITL

### Requirements (ignore if you are going to use Docker)
You will need all these libraries:

- Python client for ROS 2 Humble:
  - Follow the standard process of instalation and setup for ROS2
  https://docs.ros.org/

- asyncio
  - Install: pip install asyncio

- MAVSDK-Python
  - Install: pip install mavsdk

- AirSim API
  - Install:
    - pip install msgpack-rpc-python
    - pip install airsim

### Usage

#### Running on Docker
You can use this application inside a Docker container. To do it,
install docker (https://docs.docker.com/engine/install/ubuntu) and build
the image for the container:
```bash
docker compose build
```
And then create configuration file for the script to use. Just create a
config.json file in the root folder of this project.
You can check what to write in this file below.

After everything is setup, run the container:
```bash
docker compose up
```

#### Running without Docker

You can also directly run it from source, go to the src folder and you will 
can use the main script directly.

Display usage:
```bash
python3 maestro.py -h
```

### Config file

#### Structure
The JSON file must be organized in the following way:
```json
{
    "name_of_drone_1": {
        "mission_path" : null
    },
    "name_of_drone_2": {
        "mission_path" : "another/mission.plan"
    },
    "name_of_drone_3": {
        "mission_path" : "path/to/file.plan"
    },
    "name_of_drone_4": {}
}
```
Using this file 4 drones would be spawned. Drone named ”name_of_drone3”
would try to execute a mission described by the path/to/file.plan file, and
drone named ”name_of_drone2” would try to execute a mission described by the
another/mission.plan file.

Notice that you can manually setup the name of the drone, inform how
many drones to be spawned and give missions for them to be executed, all in
one configuration file. It also allows us to implement new features using the
same configuration file.

#### -c, --config
You can point to a configuration file using this option, it can be written in
any of the forms, like this:
```bash
python3 maestro.py -c path/to/config.json
# or
python3 maestro.py --config path/to/config.json
```

#### Relative path
If in the directory that you are executing the script, exists a config.json
file, and no other configuration file was issued to the script using -c or
–config, the script will try to read that file as configuration.

#### Absolute path
If none of the situations above happen, maestro.py will look for config.json
file in the folder of the script itself.


### AirSim with external physics engine
You can use this script to update the position of an AirSim drone based on info
available by PX4 SITL, without using the physics engine of AirSim.

To do it, there are some details that must be highlighted.
#### Name of the drones
The names defined for the drones in AirSim configuration file, must be the same
for the drones in the configuration file of this script.

For instance, maestro would use a configuration file like this:
```json
{
    "CoolDrone1": {
        "mission_path" : null
    },
    "CoolDrone2": {
        "mission_path" : "another/mission.plan"
    }
}
```

And AirSim would use a settings file like this (remember to set the
"PhysicsEngineName" accordingly):
```json
{
    "SettingsVersion": 1.2,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "PhysicsEngineName":"ExternalPhysicsEngine",
    "Vehicles": {
        "CoolDrone1": {
            "VehicleType": "PX4Multirotor",
            "LockStep": true,
            "X": 0, "Y": 0, "Z": 0
        },
        "CoolDrone2": {
            "VehicleType": "PX4Multirotor",
            "LockStep": true,
            "X": 0, "Y": 4, "Z": 0
        }
    }
}
```

#### -a, --airsim-external
After you have done the configuration appropriately, you must run AirSim and
some other physics simulator for PX4 SITL to bind to.

Along with this script with this flag set:
```bash
python3 maestro.py -a
```