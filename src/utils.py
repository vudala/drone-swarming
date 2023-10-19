# standard
import pickle
import math
import time

# 3rd party
from mavsdk.telemetry import Position, VelocityNed


# Assumptions and parameters initialization for battery
P_PIXHAWK = 2.2  # Power consumption of the Pixhawk in watts
C_BATTERY = 22.0   # Battery capacity in Ah
V_BATTETY_FULL = 25.2  # Fully charged voltage of a 6S LiPo battery in volts
E_BATTERY = C_BATTERY * V_BATTETY_FULL  # Total energy of the battery in Wh

# Coefficients Initialization fro battery
K_T = 1.5  # Throttle coefficient in W/%
K_GS = 3.33  # Ground speed coefficient in W/(m/s)
K_CR = 15.0  # Climb rate coefficient in W/(m/s)


def obj_to_bytearray(obj: object):
    """
    Turns an Object into a byte list to be used as ByteMultiArray by ROS2

    Parameters
    ----------
    - obj: Object
        - Any object

    Return
    ------
    - data: list
    """
    blob = pickle.dumps(obj)
    data = []
    for x in blob:
        data.append(x.to_bytes(1, 'little'))
    return data


def bytearray_to_obj(arr: list):
    """
    Turns a byte list back to an object

    Parameters
    ----------
    - arr: list
        - Array of bytes

    Return
    ------
    - obj: Object
        - An unserialized object
    """
    blob = b''.join(arr)
    return pickle.loads(blob)


def latlonhei_to_xyz(lat_rad: float, lon_rad: float, hei_cm: float):
    """
    Convert latitude, longitude and height to XYZ coordinates

    Parameters
    ----------
    - lat_rad: float
        - Latitude in radians
    - lon_rad: float
        - Longitude in radians
    - hei_cm: float
        - Height in centimeters

    Return
    ------
    - (x, y, z): (float, float, float) 
    """
    EARTH_RADIUS_CENTIMENTERS = 637100000.0
    R = EARTH_RADIUS_CENTIMENTERS

    x = (R + hei_cm) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (R + hei_cm) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (R + hei_cm) * math.sin(lat_rad)

    return x, y, z


def distance_cm(pos1: Position, pos2: Position):
    """
    Calculates the distance between two points in centimeters

    Parameters
    ----------
    - pos1: Position
        - First point
    - pos2: Position
        - Second point

    Return
    ------
    - distance: float
        - Distance in centimeters
    """
    lat1, lat2 = (pos1.latitude_deg, pos2.latitude_deg)
    lon1, lon2 = (pos1.longitude_deg, pos2.longitude_deg)
    hei1, hei2 = (pos1.absolute_altitude_m, pos2.absolute_altitude_m)

    lat1, lat2 = (math.radians(lat1), math.radians(lat2))
    lon1, lon2 = (math.radians(lon1), math.radians(lon2))

    x1, y1, z1 = latlonhei_to_xyz(lat1, lon1, hei1 * 100.0)
    x2, y2, z2 = latlonhei_to_xyz(lat2, lon2, hei2 * 100.0)

    d = math.sqrt(
        math.pow((x2 - x1), 2.0) +
        math.pow((y2 - y1), 2.0) +
        math.pow((z2 - z1), 2.0)
    )

    return d


def ground_speed_ms(vel: VelocityNed):
    """
    Returns the ground speed in m/s using the NED velocity of the drone

    Parameters
    ----------
    - vel: VelocityNED
        - NED velocity of the drone
    
    Return
    ------
    - gnd_speed: float
    """
    return math.sqrt(
        math.pow(vel.north_m_s, 2.0) + math.pow(vel.east_m_s, 2.0)
    )