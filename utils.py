import pickle


def obj_to_bytearray(obj: object):
    """
    Turns an Object into a byte list to be used as ByteMultiArray by ROS2
    """
    blob = pickle.dumps(obj)
    data = []
    for x in blob:
        data.append(x.to_bytes(1, 'little'))
    return data


def bytearray_to_obj(arr: list):
    """
    Turns a byte list back to an object
    """
    blob = b''.join(arr)
    return pickle.loads(blob)
