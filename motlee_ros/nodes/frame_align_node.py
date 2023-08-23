#!/usr/bin/env python3

import numpy as np
from scipy.spatial.transform import Rotation as Rot

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
        self.neighbor_robots = rospy.get_param("~neighbor_robots", ["1", "2", "3"])
        ts = rospy.get_param("~ts", 1.) # frame-align frequency
        run_filter = rospy.get_param("~filter", True) # filter or take a single measurement
        num_objs_req = rospy.get_param("~num_objs_req", 15) # number of associations required to perform frame alignment
        clipper_eps = rospy.get_param("~clipper_eps", .3) # CLIPPER epsilon parameter
        clipper_sig = rospy.get_param("~clipper_sig", .3) # CLIPPER sigma parameter

        # Set up frame aligner from MOTLEE
        self.frame_align_filter = FrameAlignFilter(
            cam_id=self.robot_id,
            connected_cams=self.neighbor_robots,
            ts=ts,
            filter_frame_align=run_filter
        )
        self.frame_aligner = FrameAligner(
            method=AssocMethod.CLIPPER,
            num_objs_req=num_objs_req,
            clipper_epsilon=clipper_eps,
            clipper_sigma=clipper_sig
        )
        all_robots = self.neighbor_robots + [self.robot_id]
        self.landmarks = {neighbor: np.array([]) for neighbor in all_robots}
        self.ages = {neighbor: np.array([]) for neighbor in all_robots}

        # ROS communication
        # Assuming all maps are published to one place, and then each neighboring robot gets its own frame_align
        # self.pub_fa =  {neighbor: rospy.Publisher(f'/frame_align/{neighbor}', motlee_msgs.SE3Transform, 10) for neighbor in neighbor_robots}
        self.pub_fa =  rospy.Publisher(f'frame_align', motlee_msgs.SE3Transform, queue_size=10)
        self.sub_map = [rospy.Subscriber(f'/{neighbor}/map', motlee_msgs.ObjArray, self.map_cb, callback_args=neighbor, queue_size=10) for neighbor in all_robots]
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_cb)
        
    def timer_cb(self, msg):
        # Run frame alignment on each neighboring robot
        for neighbor in self.neighbor_robots:
            sol = self.frame_aligner.align_objects(static_objects=[self.landmarks[self.robot_id], self.landmarks[neighbor]], 
                                                   static_ages=[self.ages[self.robot_id], self.ages[neighbor]])
            self.frame_align_filter.update_transform(neighbor, sol)
            
            T_msg = motlee_msgs.SE3Transform()
            T_msg.transform = self.frame_align_filter.transforms[neighbor].reshape(-1)
            T_msg.frame_src = neighbor
            T_msg.frame_dest = self.robot_id
            T_msg.header.stamp = rospy.Time.now()

            self.pub_fa.publish(T_msg)
    
    def map_cb(self, msg, robot_id):
        # update map storage
        self.landmarks[robot_id] = np.array([[o.position.x, o.position.y, o.position.z] for o in msg.objects])
        self.ages[robot_id] = np.array([o.ell for o in msg.objects])
        


if __name__ == "__main__":
    rospy.init_node("frame_aligner")

    frame_aligner = FrameAlignerNode()
    rospy.spin()