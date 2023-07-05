import numpy as np
from numpy.linalg import norm, inv
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
import message_filters

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry

from motlee_msgs.msg import ObjArray, Obj

from motlee.mot.multi_object_tracker import MultiObjectTracker
from motlee.config.rover_mot_params import RoverMOTParams as MOTParams
from motlee.config.track_params import ConeParams as StaticObjParams
from motlee.utils.transform import transform, pos_quat_to_transform

from .utils import T_FLURDF, pose_msg_2_T

class MapperNode(Node):
    
    def __init__(self):
        super().__init__('mapper')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('tau_local', 2.0),
                ('tau_global', .75),
                ('alpha', 2000),
                ('kappa', 2000),
                ('n_meas_to_init_track', 3),
                ('robot_id', 0),
                ('pose_from_odom', True),
                ('pose_topic', '/t265/odom/sample'),
                ('obj_names', [
                    'traffic light',
                    'fire hydrant', 
                    'bicycle',
                    'stop sign',
                    'parking meter',
                    'bench',
                    'chair',
                    'pottedplant',
                ]),
            ]
        )
        
        mot_params = MOTParams()
        mot_params.Tau_LDA = self.get_parameter('tau_local').value
        mot_params.Tau_GDA = self.get_parameter('tau_global').value
        mot_params.alpha = self.get_parameter('alpha').value
        mot_params.kappa = self.get_parameter('kappa').value
        mot_params.n_meas_to_init_track = self.get_parameter('n_meas_to_init_track').value
        robot_id = self.get_parameter('robot_id').value
        pose_from_odom = self.get_parameter('pose_from_odom').value
        pose_topic = self.get_parameter('pose_topic').value
        self.obj_names = self.get_parameter('obj_names').value

        # MOT setup
        static_obj_params = StaticObjParams()
        static_obj_params.Q = .05*np.eye(2)
        self.mots = {
            name: MultiObjectTracker(camera_id=robot_id, connected_cams=[], 
                                     params=mot_params, track_params=StaticObjParams()) 
            for name in self.obj_names
        }
        
        # pubs subs
        self.pub_map = self.create_publisher(ObjArray, 'object_map', 10)
        self.pub_map_poses_only = self.create_publisher(PoseArray, 'object_map/poses_only', 10)
        
        pose_type = Odometry if pose_from_odom else PoseStamped
        subs = []
        subs.append(message_filters.Subscriber(self, pose_type, pose_topic)) # odometry
        subs.append(message_filters.Subscriber(self, ObjArray, f'detections/static'))
        self.time_sync = message_filters.ApproximateTimeSynchronizer(subs, 1000, .05)
        self.time_sync.registerCallback(self.dets_cb)
    
    def dets_cb(self, *msgs):
        # print('cb')
        pose_msg = msgs[0]
        dets_msg = msgs[1]
        
        T_WB = pose_msg_2_T(pose_msg.pose.pose)
        R_WB = T_WB[:3,:3]
        # T_WC = T_WB @ T_FLURDF
        dets = [transform(T_WB, np.array([[det.position.x], [det.position.y], [det.position.z]]))[:2,:] for det in dets_msg.objects]
        # print([np.array(obj.covariance).reshape((3,3)) for obj in dets_msg.objects])
        Rs = [(R_WB @ np.array(obj.covariance).reshape((3,3)) @ R_WB.T)[:2,:2] for obj in dets_msg.objects]
        names = [obj.class_name for obj in dets_msg.objects]
        
        for on in self.obj_names:
            # print(on)
            # print([det for det, name in zip(dets, names) if name == on])
            # print([R for R, name in zip(Rs, names) if name == on])
            self.mots[on].local_data_association(
                [det for det, name in zip(dets, names) if name == on], 
                feature_vecs=np.arange(len(dets)),
                Rs=[R for R, name in zip(Rs, names) if name == on])
            self.mots[on].dkf()
            self.mots[on].track_manager()
        
        map = ObjArray()
        map.header = pose_msg.header
        for name in self.mots:
            # print(name)
            # for new_lnmrk in self.mots[name].new_tracks:
            #     print(new_lnmrk.P)
            #     print(new_lnmrk.ell)
            for landmark in self.mots[name].tracks:
                obj = Obj()
                obj.position.x, obj.position.y = landmark.state.reshape(-1)
                obj.covariance[0] = landmark.P[0,0]
                obj.covariance[4] = landmark.P[1,1]
                obj.class_name = name
                map.objects.append(obj)

        self.pub_map.publish(map)
        
        map_poses_only = PoseArray()
        map_poses_only.header = pose_msg.header
        for obj in map.objects:
            pose = Pose()
            pose.position = obj.position
            pose.orientation.x, pose.orientation.y, \
                pose.orientation.z, pose.orientation.w = \
                Rot.from_euler('xyz', [0., -90., 0.], degrees=True).as_quat()
            map_poses_only.poses.append(pose)

        return

def main(args=None):
    rclpy.init(args=args)
    mapper = MapperNode()
    rclpy.spin(mapper)
    
    mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
