#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial import KDTree
import message_filters
import rospy
import tf

import geometry_msgs.msg as geometry_msgs
import motlee_msgs.msg as motlee_msgs

from motlee.mot.multi_object_tracker import MultiObjectTracker
from motlee.mot.motion_model import MotionModel
from motlee.utils.transform import transform

from motlee.realign.frame_align_filter import FrameAlignFilter
from motlee.realign.frame_aligner import FrameAligner, AssocMethod

from utils import pose_msg_2_T


class FrameAlignerNode:
    """
    Node used for aligning the odom0 initial odometry frame with the drifting odom frame using static landmarks.
    
    landmarks_saved denotes landmarks loaded from a saved map.
    landmarks_rec denote recently detected landmarks.
    
    
    TF Tree:
    
    F_robot
    |      > odometry
    F_odom
    |      > frame align (estimated by this node)
    F_odom0
    |      > odom init
    F_world
    
    """
    def __init__(self):
        
        # ROS Parameters
        self.robot_id = rospy.get_param("~robot_id", "0")
        ts = rospy.get_param("~ts", 1.) # frame-align frequency
        run_filter = rospy.get_param("~filter", True) # filter or take a single measurement
        num_objs_req = rospy.get_param("~num_objs_req", 10) # number of associations required to perform frame alignment
        clipper_eps = rospy.get_param("~clipper_eps", .3) # CLIPPER epsilon parameter
        clipper_sig = rospy.get_param("~clipper_sig", .3) # CLIPPER sigma parameter

        map_path = rospy.get_param("~map_path")
        # print(map_path)
        self.saved_map = self.read_map(map_path)
        self.ldmrks_saved_world, self.ldmrks_saved_wh = self.get_landmarks_info_from_map(self.saved_map)

        self.T_w_odom0 = None
        
        # Set up frame aligner from MOTLEE
        self.frame_align_filter = FrameAlignFilter(
            cam_id='map',
            connected_cams=[self.robot_id],
            ts=ts,
            filter_frame_align=run_filter
        )
        self.frame_aligner = FrameAligner(
            method=AssocMethod.CLIPPER,
            num_objs_req=num_objs_req,
            clipper_epsilon=clipper_eps,
            clipper_sigma=clipper_sig
        )
        self.ldmrks_rec_odom = np.array([]) # recent landmarks in frame odom0
        self.ages = np.array([])

        # ROS communication
        # Assuming all maps are published to one place, and then each neighboring robot gets its own frame_align
        self.pub_fa =  rospy.Publisher(f'/{self.robot_id}/frame_align', motlee_msgs.SE3Transform, queue_size=10)
        self.sub_map = rospy.Subscriber(f'/{self.robot_id}/recent_map', motlee_msgs.ObjArray, self.map_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("Initialize frame alignment node.")

    def get_landmarks_info_from_map(self, map):
        # landmarks start out in world frame, we will read them in world frame, transform them later
        landmarks = []
        landmarks_size = []
        for _, info in map.items():
            landmarks.append(info[2:])
            landmarks_size.append(info[0:2])
        return np.array(landmarks), np.array(landmarks_size)
    
    def read_map(self, path):
        # landmarks start out in world frame, we will read them in world frame, transform them later
        f = open(path, 'r')
        lines = f.readlines()
        
        obj_map = {}
        
        for i in range(1, len(lines)):
            substrings = lines[i].split(", ")
            obj_info = np.array([float(s) for s in substrings])
            obj_map[obj_info[0]] = obj_info[1:] 
        f.close()
        return obj_map

    def filter_landmarks(self, radius=10.0, width_threshold=3.0, height_threshold=8.0):
        # print(self.ldmrks_rec_odom)
        # TODO: use widths and heights to filter out putative associations
        # landmarks_KDTree is a KD tree in the odom0, so should transform ldmrks_rec_odom into odom0 before fetching
        ldmrks_recent_odom0 = transform(self.T_odom0_odom, self.ldmrks_rec_odom)
        ii = self.ldmrks_saved_odom0_KD.query_ball_point(ldmrks_recent_odom0, r=radius)        

        # print("Indices: ", ii)
        indices_filtered = []
        for i in range(len(np.array(ii))):
            lm_size = self.ldmrks_rec_wh[i]
            for j in ii[i]:
                obj_candidate_size = self.ldmrks_saved_wh[j]
                if abs(lm_size[0] - obj_candidate_size[0]) < width_threshold and abs(lm_size[1] - obj_candidate_size[1]) < height_threshold:
                    indices_filtered.append(j)

        indices_filtered = list(set(indices_filtered))
        # print("Filtered indices: ", indices_filtered)
        return self.ldmrks_saved_odom0[indices_filtered, :]
    
    def timer_cb(self, msg):
        if self.T_w_odom0 is None:
            try:
                # TODO: rename when we figure out what we want the tf tree to look like exactly
                (t, q) = self.tf_listener.lookupTransform('/world', '/odom0', rospy.Time(0))
                self.T_w_odom0 = np.eye(4)
                self.T_w_odom0[:3,:3] = Rot.from_quat(q).as_matrix()
                self.T_w_odom0[:3,3] = t
                
                # saved_landmarks are in the ODOM0 FRAME
                self.ldmrks_saved_odom0 = transform(np.linalg.inv(self.T_w_odom0), self.ldmrks_saved_world)
                self.ldmrks_saved_odom0_KD = KDTree(self.ldmrks_saved_odom0)
            except:
                return
        if len(self.ldmrks_rec_odom) == 0:
            return
        # print(self.ldmrks_saved_odom0.shape)
        print("Perceived landmarks num: ", self.ldmrks_rec_odom.shape)

        landmarks_filtered = self.filter_landmarks()
        print("Filtered landmarks num: ", landmarks_filtered.shape)
        if len(landmarks_filtered) == 0:
            # print("No filtered result.")
            return
        # we want transformation from odom to odom0
        # TODO: filter out width/height using putative associations
        sol = self.frame_aligner.align_objects(static_objects=[landmarks_filtered, self.ldmrks_rec_odom])
        print("Alignment result: ", sol)
        self.frame_align_filter.update_transform(self.robot_id, sol)
            
        T_msg = motlee_msgs.SE3Transform()
        T_msg.transform = self.frame_align_filter.transforms[self.robot_id].reshape(-1)
        T_msg.frame_src = self.robot_id
        T_msg.frame_dest = 'map'
        T_msg.header.stamp = rospy.Time.now()

        self.pub_fa.publish(T_msg)
    
    def map_cb(self, msg):
        # landmarks come in odom frame, we want to use them to estimate the transformation from odom frame to odom0 frame
        #                 
        #       F_robot
        #       |      > odometry
        #       F_odom
        #       |      > frame align (estimated by this node)
        #       F_odom0
        #       |      > odom init
        #       F_world
        #
        # update map storage
        landmarks = np.array([[o.position.x, o.position.y, o.position.z] for o in msg.objects])
        self.ldmrks_rec_odom = transform(self.T_odom0_odom, landmarks)
        self.ldmrks_rec_wh = np.array([[o.width, o.height] for o in msg.objects])
        self.ages = np.array([o.ell for o in msg.objects])
    
    @property
    def T_odom0_odom(self):
        """Transfrom from odom frame to odom0 frame
        """
        self.frame_align_filter.transforms[self.robot_id]
        


if __name__ == "__main__":
    rospy.init_node("frame_aligner")
    frame_aligner = FrameAlignerNode()
    rospy.spin()