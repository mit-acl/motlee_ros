import numpy as np
from numpy.linalg import norm, inv
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation as Rot

import rclpy
from rclpy.node import Node
import message_filters

from geometry_msgs.msg import PoseArray, Pose, PoseStamped, PoseWithCovariance
from nav_msgs.msg import Odometry

from motlee_msgs.msg import ObjArray, Obj, SE3Transform

from motlee.mot.multi_object_tracker import MultiObjectTracker
from motlee.config.rover_mot_params import RoverMOTParams as MOTParams
from motlee.config.track_params import ConeParams as StaticObjParams
from motlee.utils.transform import transform, pos_quat_to_transform
from motlee.realign.frame_align_filter import FrameAlignFilter
from motlee.realign.frame_aligner import FrameAligner, AssocMethod

from .utils import T_FLURDF, pose_msg_2_T

class FrameAlignNode(Node):
    
    def __init__(self):
        super().__init__('frame_align_node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_i', 'RR01'),
                ('robot_j', 'RR04'),
                ('ts', 1.),
                ('filter', True),
                ('num_objs_req', 8),
                ('icp_max_dist', 1.0),
            ]
        )
        robot_i = self.get_parameter('robot_i').value
        robot_j = self.get_parameter('robot_j').value
        ts = self.get_parameter('ts').value
        filter_frame_align = self.get_parameter('filter').value
        num_objs_req = self.get_parameter('num_objs_req').value
        icp_max_dist = self.get_parameter('icp_max_dist').value
        
        # set up frame aligner
        self.frame_align_filter = FrameAlignFilter(
            cam_id=robot_i,
            connected_cams=[robot_j],
            ts=ts,
            filter_frame_align=filter_frame_align
        )
        self.frame_aligner = FrameAligner(
            method=AssocMethod.ICP_STRONG_CORRES,
            num_objs_req=num_objs_req,
            icp_max_dist=icp_max_dist,
        )
        self.robot_j = robot_j
        
        # pubs subs
        self.pub_fa = self.create_publisher(SE3Transform, f'{robot_i}/frame_align/{robot_j}', 10)
        
        subs = []
        subs.append(message_filters.Subscriber(self, ObjArray, f'{robot_i}/object_map'))
        subs.append(message_filters.Subscriber(self, ObjArray, f'{robot_j}/object_map'))
        self.time_sync = message_filters.ApproximateTimeSynchronizer(subs, 1000, .5)
        self.time_sync.registerCallback(self.maps_cb)
    
    def maps_cb(self, *msgs):
        map_i_msg = msgs[0]
        map_j_msg = msgs[1]
        
        T_current = self.frame_align_filter.transforms[self.robot_j]
        objs1 = np.array([[o.position.x, o.position.y, o.position.z] for o in map_i_msg.objects])
        objs2 = np.array([[o.position.x, o.position.y, o.position.z] for o in map_j_msg.objects])
        ages1 = np.array([o.ell for o in map_i_msg.objects])
        ages2 = np.array([o.ell for o in map_j_msg.objects])
        
        sol = self.frame_aligner.align_objects(objs1, objs2, ages1, ages2, T_current)
        self.frame_align_filter.update_transform(self.robot_j, sol)
        
        T_msg = SE3Transform()
        T_msg.transform = self.frame_align_filter.transforms[self.robot_j].reshape(-1)

        self.pub_fa.publish(T_msg)

        return

def main(args=None):
    rclpy.init(args=args)
    frame_align = FrameAlignNode()
    rclpy.spin(frame_align)
    
    frame_align.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
