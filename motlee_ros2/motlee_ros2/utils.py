import numpy as np

from std_msgs.msg import Header

T_FLURDF = np.array([
    [0, 0, 1, 0],
    [-1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
], dtype=np.float64)

T_RDFFLU = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 0],
    [1, 0, 0, 0],
    [0, 0, 0, 1]
], dtype=np.float64)

def stamp_to_time(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 10**(-9)

def time_to_stamp(time):
    msg = Header()
    msg.stamp.sec = int(time)
    msg.stamp.nanosec = int((time % 1) * 10**9)
    return msg.stamp

def track_id_to_tuple(track_id):
    return (track_id.robot_id, track_id.track_id)