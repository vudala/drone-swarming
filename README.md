# Maestro
Implementing a drone swarm controller on PX4 SITL

### Requirements
You will need all these libraries:

- Python client for ROS2:
  - Follow the standard process of instalation and setup for ROS2
  https://docs.ros.org/

- asyncio
  - Install: pip install asyncio

- MAVSDK-Python
  - Install: pip install mavsdk

The requirements needs better documentation

Or you can run
```bash
pip install -r requirements.txt
```
to install mavsdk and asyncio

### Usage
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
would try to execute a mission described by the path/to/file.plan file, and drone
named ”name_of_drone2” would try to execute a mission described by the an-
other/mission.plan file.

Notice that you can manually setup the name of the drone, inform how
many drones to be spawned and give missions for them to be executed, all in
one configuration file. It also allows us to implement new features using the
same configuration file.

#### -c, --config
You can point to a configuration file using this option, it can be written in any
of the forms, like this:
```bash
python3 maestro.py -c path/to/config.json
# or
python3 maestro.py --config path/to/config.json
```

#### Relative path
If in the directory that you are executing the script, exists a config.json file,
and no other configuration file was issued to the script using -c or –config, the
script will try to read that file as configuration.

#### Absolute path
If none of the situations above happen, maestro.py will look for config.json file
in the folder of the script itself.
