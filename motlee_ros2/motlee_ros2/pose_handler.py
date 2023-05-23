import numpy as np

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class PoseHandler():
    
    def __init__(self, max_pose_len=100):        
        self.poses = []
        self.pose_times = []
        self.max_pose_len = max_pose_len

    def pose_cb(self, msg):
        '''
        Adds the most recent pose estimate to the list of poses.
        '''
        if type(msg) == PoseStamped:
            pose = msg.pose
        elif type(msg) == Odometry:
            pose = msg.pose.pose
        pose = np.array([
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])
        self.poses.append(pose)
        self.pose_times.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9))

        self.poses = self.poses[-self.max_pose_len:]
        self.pose_times = self.pose_times[-self.max_pose_len:]
        
        return
    
    def get_pose_from_time(self, t, allowable_diff=0.1):
        '''
        Returns the stored pose that has the timestamp closest to the input time t.
        allowable_diff is the difference in seconds that the returned pose is allowed to be from
        the desired t.
        Pose is 7-dimensional array containing position in R3 and orientation as a quaternion.
        '''
        pose_times = np.array(self.pose_times)
        ge_list = np.where(pose_times >= t)[0]
        lt_list = np.where(pose_times < t)[0]
        
        assert len(ge_list) > 0 or len(lt_list) > 0, f'time: {t}\npose_times: {pose_times}'
        if len(ge_list) == 0:
            pose_idx = lt_list[-1]
        elif len(lt_list) == 0:
            pose_idx = ge_list[0]
        else:
            if np.abs(t - self.pose_times[ge_list[0]]) < \
                np.abs(t - self.pose_times[lt_list[-1]]):
                pose_idx = ge_list[0]
            else:
                pose_idx = lt_list[-1]
        pose = self.poses[pose_idx]
        if np.abs(t - self.pose_times[pose_idx]) > allowable_diff:
            return None
        return pose
    
    def __len__(self):
        return len(self.poses)