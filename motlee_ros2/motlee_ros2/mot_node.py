import numpy as np
from numpy.linalg import norm, inv
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry

from centertrack_msgs.msg import ObjDet3DArray
from motlee_msgs.msg import TrackStateArray, TrackState

from motlee.mot.multi_object_tracker import MultiObjectTracker
from motlee.config.rover_mot_params import RoverMOTParams
from motlee.config.track_params import TrackParams
from motlee.utils.transform import transform, pos_quat_to_transform

class MultiObjectTrackerNode(Node):
    
    def __init__(self):
        super().__init__('multi_object_tracker')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tau_local', .8),
                ('tau_global', .75),
                ('alpha', 2000),
                ('kappa', 10),
                ('n_meas_to_init_track', 5),
                ('robot_id', 0),
                ('pose_from_odom', True),
                ('pose_topic', '/t265/odom/sample'),
                ('mot_dt', .04)
            ]
        )
        
        mot_params = RoverMOTParams()
        mot_params.Tau_LDA = self.get_parameter('tau_local').value
        mot_params.Tau_GDA = self.get_parameter('tau_global').value
        mot_params.alpha = self.get_parameter('alpha').value
        mot_params.kappa = self.get_parameter('kappa').value
        mot_params.n_meas_to_init_track = self.get_parameter('n_meas_to_init_track').value
        robot_id = self.get_parameter('robot_id').value
        pose_from_odom = self.get_parameter('pose_from_odom').value
        pose_topic = self.get_parameter('pose_topic').value
        timer_period = self.get_parameter('mot_dt').value
        print(f'MOT dt: {timer_period}')

        # MOT setup
        self.mot = MultiObjectTracker(camera_id=robot_id, connected_cams=[], params=mot_params, track_params=TrackParams())
        
        # eventually publish and subscribe to other nodes
        self.track_publisher = self.create_publisher(TrackStateArray, 'tracks', 10)
        self.track_poses_publisher = self.create_publisher(PoseArray, 'track_poses', 10)
        
        self.timer = self.create_timer(timer_period, self.timer_cb)
        
        pose_type = Odometry if pose_from_odom else PoseStamped
        self.pose_sub = self.create_subscription(pose_type, pose_topic, self.pose_cb, 10)
        self.dets_sub = self.create_subscription(ObjDet3DArray, 'detections', self.dets_cb, 10)
        # self.dets_sub = self.create_subscription(ObjDet3DArray, 'detections_transformed', self.dets_cb, 10)
        # TODO: Eventually, I would like MOT node to not know about its pose and have another node listen to
        # detections and republish them after transforming them
        
        self.poses = []
        self.pose_times = []
        self.max_pose_len = 100

        self.detections = []
        self.detections_Rs = []
        self.detections_t = None

        self.T_BC = np.array([
            [0, 0, 1, 0],
            [-1, 0, 0, 0],
            [0, -1, 0, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)

    def timer_cb(self):
        '''
        Track prediction, detection data association, and measurement updates.
        '''
        if len(self.poses) == 0 or self.detections_t is None:
            return
        self.mot.local_data_association(self.detections, feature_vecs=np.arange(len(self.detections)), Rs=self.detections_Rs)
        self.detections, self.detections_Rs = [], []
        # TODO: get feature vecs out of here
        
        # TODO: I think I will do the add observations before running the next cycle of local da, so the local da should really
        # go at the end of this loop
        observations = self.mot.get_observations()
        self.mot.add_observations([obs for obs in observations if  obs.destination == self.mot.camera_id])
        self.mot.dkf()
        self.mot.track_manager()

        track_pose_array = PoseArray()
        track_pose_array.header.frame_id = 'world'
        for track in self.mot.tracks:
            track_pose = Pose()
            track_pose.position.x, track_pose.position.y = track.state.item(0), track.state.item(1)
            # theta = np.arctan2(track.state.item(3), track.state.item(2))
            # pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = \
            #     Rot.from_euler('xyz', [0, 0, theta]).as_quat().tolist()
            track_pose_array.poses.append(track_pose)
        self.track_poses_publisher.publish(track_pose_array)

        tracks_msg = TrackStateArray()
        tracks_msg.header.frame_id = 'world'
        tracks_msg.header.stamp.sec = int(self.detections_t)
        tracks_msg.header.stamp.nanosec = int((self.detections_t % 1) * 10**9)
        for track in self.mot.tracks:
            track_state = TrackState()
            track_state.track_id.robot_id, track_state.track_id.track_id = track.id
            track_state.position.x, track_state.position.y = track.state.item(0), track.state.item(1)
            track_state.velocity.x, track_state.velocity.y = track.state.item(2), track.state.item(3)
            tracks_msg.tracks.append(track_state)
        self.track_publisher.publish(tracks_msg)
            
    def pose_cb(self, msg):
        '''
        Saves most recent pose estimate.
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
        # print(self.poses[0])
        
        return
    
    def dets_cb(self, msg):
        # wait for initial pose estimate
        # self.last_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9)
        if len(self.poses) == 0:
            return
        self.detections_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 10**(-9)
        self.detections = []
        for detection in msg.detections:
            position_c = np.array([detection.pose.pose.position.x, 
                                    detection.pose.pose.position.y, 
                                    detection.pose.pose.position.z])
            position_b = transform(self.T_BC, position_c)
            
            # find the pose with the closest timestamp to the object detections
            pose_times = np.array(self.pose_times)
            ge_list = np.where(pose_times >= self.detections_t)[0]
            lt_list = np.where(pose_times < self.detections_t)[0]
            
            assert len(ge_list) > 0 or len(lt_list) > 0, f'detection_time: {self.detections_t}\npose_times: {pose_times}'
            if len(ge_list) == 0:
                pose_idx = lt_list[-1]
            elif len(lt_list) == 0:
                pose_idx = ge_list[0]
            else:
                if np.abs(self.detections_t - self.pose_times[ge_list[0]]) < \
                    np.abs(self.detections_t - self.pose_times[lt_list[-1]]):
                    pose_idx = ge_list[0]
                else:
                    pose_idx = lt_list[-1]
            pose = self.poses[pose_idx]
            
            T_LB = pos_quat_to_transform(pose[:3], pose[3:])
            position_ell = transform(T_LB, position_b)
            self.detections.append(position_ell[:2].reshape((2, 1)))

            # Covariance
            obj_dist = norm(position_b)
            sigma_sq = 1.0 + (obj_dist > 10.0) * 0.2 * (obj_dist - 10.0)
            R = np.diag([sigma_sq, sigma_sq])
            self.detections_Rs.append(R)
            #TODO: send out detections here
            # print(position_ell)
            
        return

def main(args=None):
    rclpy.init(args=args)
    mot = MultiObjectTrackerNode()
    rclpy.spin(mot)
    
    mot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
