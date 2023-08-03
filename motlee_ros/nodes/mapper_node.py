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

from utils import pose_msg_2_T

class Mapper:
    def __init__(self):
        
        # ROS Parameters
        robot_id = rospy.get_param("~robot_id", "0")
        Q = rospy.get_param("~Q", [[.25**2, 0.], [0., .25**2]]) # process noise covariance
        P0 = rospy.get_param("~P0", [[.5**2, 0.], [0., .5**2]]) # initial estimate covariance
        tau = rospy.get_param("~tau", 2.) # mahalanobis distance gate for landmark data association
        kappa = rospy.get_param("~kappa", 100) # number of timesteps to keep a landmark without new measurements before deleting
        nu = rospy.get_param("~nu", 3) # number of inital consecutive measurements required to create a new object
        self.dim = rospy.get_param("~dimension", 2) # 2 or 3 - dimension of landmarks

        # Set up static landmark motion model
        landmark_model = MotionModel(A=np.eye(self.dim), H=np.eye(self.dim), Q=Q, R=np.array([]), P0=P0)

        # Set up mapper from MOTLEE
        self.mapper = MultiObjectTracker(
            camera_id=robot_id,
            connected_cams=[],
            track_motion_model=landmark_model,
            tau_local=tau,
            tau_global=0.,
            alpha=2000,
            kappa=kappa,
            nu=nu,
            track_storage_size=1
        )

        # ROS communication
        self.sub_det = rospy.Subscriber('detections', motlee_msgs.ObjArray, self.det_cb, queue_size=2)
        self.pub_map = rospy.Publisher('map', motlee_msgs.ObjArray, queue_size=10)
        self.pub_map_poses_only = rospy.Publisher('map/poses_only', geometry_msgs.PoseArray, queue_size=10)

    def det_cb(self, dets_msg):
        """Detection callback - incorporates new measurements into map

        Args:
            dets_msg (motlee_msgs/ObjArray): landmark measurements and covariances in world frame
        """
        
        dets = [np.array([[det.position.x], [det.position.y], [det.position.z]])[:self.dim,:] for det in dets_msg.objects]
        Rs = [np.array(obj.covariance).reshape((3,3))[:self.dim,:self.dim] for obj in dets_msg.objects]
        
        self.mapper.local_data_association(
            [det for det in dets], 
            feature_vecs=np.arange(len(dets)),
            Rs=[R for R in Rs])
        self.mapper.dkf()
        self.mapper.track_manager()
        
        map = motlee_msgs.ObjArray()
        map.header = dets_msg.header

        for landmark in self.mapper.tracks:
            obj = motlee_msgs.Obj()
            obj.position.x, obj.position.y = landmark.state.reshape(-1)[:self.dim]
            obj.position.z = 0 if self.dim == 2 else landmark.state.item(2)
            obj.covariance[0] = landmark.P[0,0]
            obj.covariance[4] = landmark.P[1,1]
            obj.covariance[8] = 0 if self.dim == 2 else landmark.P[2,2]
            map.objects.append(obj)

        self.pub_map.publish(map)
        
        map_poses_only = geometry_msgs.PoseArray()
        map_poses_only.header = dets_msg.header
        for obj in map.objects:
            pose = geometry_msgs.Pose()
            pose.position = obj.position
            pose.orientation.x, pose.orientation.y, \
                pose.orientation.z, pose.orientation.w = \
                Rot.from_euler('xyz', [0., -90., 0.], degrees=True).as_quat()
            map_poses_only.poses.append(pose)
        self.pub_map_poses_only.publish(map_poses_only)

        return


if __name__ == "__main__":
    rospy.init_node("mapper")

    mapper = Mapper()
    rospy.spin()