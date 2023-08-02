import numpy as np
from scipy.spatial.transform import Rotation as Rot

from std_msgs.msg import Header

# from camera to body FLU coordinates transform
T_FLURDF = np.array([
    [0, 0, 1, 0],
    [-1, 0, 0, 0],
    [0, -1, 0, 0],
    [0, 0, 0, 1]
], dtype=np.float64)

# from body FLU to camera coordinates transform
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

def pose_msg_2_T(pose_msg):
    R = Rot.from_quat([pose_msg.orientation.x, pose_msg.orientation.y, \
        pose_msg.orientation.z, pose_msg.orientation.w])
    t = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    T = np.eye(4); T[:3,:3] = R.as_matrix(); T[:3,3] = t
    return T