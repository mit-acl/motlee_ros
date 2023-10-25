#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial import KDTree
import message_filters
import rospy

import geometry_msgs.msg as geometry_msgs
import motlee_msgs.msg as motlee_msgs

from motlee.mot.multi_object_tracker import MultiObjectTracker
from motlee.mot.motion_model import MotionModel
from motlee.utils.transform import transform

from motlee.realign.frame_align_filter import FrameAlignFilter
from motlee.realign.frame_aligner import FrameAligner, AssocMethod

from utils import pose_msg_2_T


class FrameAlignerNode:
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
        self.saved_landmarks, self.saved_landmarks_size = self.get_landmarks_info_from_map(self.saved_map)
        # print(self.saved_landmarks_size)
        self.landmakrs_KDTree = KDTree(self.saved_landmarks)
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
        self.landmarks = np.array([])
        self.ages = np.array([])

        # ROS communication
        # Assuming all maps are published to one place, and then each neighboring robot gets its own frame_align
        self.pub_fa =  rospy.Publisher(f'/{self.robot_id}/frame_align', motlee_msgs.SE3Transform, queue_size=10)
        self.sub_map = rospy.Subscriber(f'/{self.robot_id}/recent_map', motlee_msgs.ObjArray, self.map_cb, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)

        rospy.loginfo("Initialize frame alignment node.")

    def get_landmarks_info_from_map(self, map):
        landmarks = []
        landmarks_size = []
        for _, info in map.items():
            landmarks.append(info[2:])
            landmarks_size.append(info[0:2])
        return np.array(landmarks), np.array(landmarks_size)
    
    def read_map(self, path):
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
        # print(self.landmarks)
        ii = self.landmakrs_KDTree.query_ball_point(self.landmarks, r=radius)
        # print("Indices: ", ii)
        indices_filtered = []
        for i in range(len(np.array(ii))):
            lm_size = self.landmarks_size[i]
            for j in ii[i]:
                obj_candidate_size = self.saved_landmarks_size[j]
                if abs(lm_size[0] - obj_candidate_size[0]) < width_threshold and abs(lm_size[1] - obj_candidate_size[1]) < height_threshold:
                    indices_filtered.append(j)

        indices_filtered = list(set(indices_filtered))
        # print("Filtered indices: ", indices_filtered)
        return self.saved_landmarks[indices_filtered, :]
    
    def timer_cb(self, msg):
        if len(self.landmarks) == 0:
            return
        # print(self.saved_landmarks.shape)
        print("Perceived landmarks num: ", self.landmarks.shape)

        landmarks_filtered = self.filter_landmarks()
        print("Filtered landmarks num: ", landmarks_filtered.shape)
        if len(landmarks_filtered) == 0:
            # print("No filtered result.")
            return
        sol = self.frame_aligner.align_objects(static_objects=[landmarks_filtered, self.landmarks])
        print("Alignment result: ", sol)
        self.frame_align_filter.update_transform(self.robot_id, sol)
            
        T_msg = motlee_msgs.SE3Transform()
        T_msg.transform = self.frame_align_filter.transforms[self.robot_id].reshape(-1)
        T_msg.frame_src = self.robot_id
        T_msg.frame_dest = 'map'
        T_msg.header.stamp = rospy.Time.now()

        self.pub_fa.publish(T_msg)
    
    def map_cb(self, msg):
        # update map storage
        self.landmarks = np.array([[o.position.x, o.position.y, o.position.z] for o in msg.objects])
        self.landmarks_size = np.array([[o.width, o.height] for o in msg.objects])
        self.ages = np.array([o.ell for o in msg.objects])
        


if __name__ == "__main__":
    rospy.init_node("frame_aligner")
    frame_aligner = FrameAlignerNode()
    rospy.spin()