import pickle


# turns an object into a byte array to be used as ByteMultiArray by ros2
def obj_to_bytearray(obj: object):
    blob = pickle.dumps(obj)
    data = []
    for x in blob:
        data.append(x.to_bytes(1, 'little'))
    return data


# turns a bytearray back to an object
def bytearray_to_obj(arr: list):
    blob = b''.join(arr)
    return pickle.loads(blob)
