#!/usr/bin/env python3

import numpy as np
from numpy.linalg import inv

import message_filters
import rospy
import cv_bridge
import cv2 as cv

import nav_msgs.msg as nav_msgs
import geometry_msgs.msg as geometry_msgs
import motlee_msgs.msg as motlee_msgs
import sensor_msgs.msg as sensor_msgs
import visualization_msgs.msg as visualization_msgs

from utils import pose_msg_2_T, T_FLURDF, xyz_2_pixel
from motlee.utils.transform import transform

class ObjVizNode:
    def __init__(self):
        
        # ROS Parameters
        self.T_BC = T_FLURDF # should make a parameter
        pose_type_str = rospy.get_param('~pose_type', 'Odometry')
        self.image_view = rospy.get_param('~image_view', True)
        self.marker_color = rospy.get_param('~marker_color', (.2, .2, .8))
        self.rdf = rospy.get_param('~rdf', False) # camera frame convention
        if pose_type_str == "Odometry":
            self.pose_type = nav_msgs.Odometry
        elif pose_type_str == "PoseStamped":
            self.pose_type = geometry_msgs.PoseStamped
        
        # internal
        self.bridge = cv_bridge.CvBridge()

        # ROS subscribers
        self.subs = [
            message_filters.Subscriber('image_raw/compressed', sensor_msgs.CompressedImage),
            message_filters.Subscriber('camera_info', sensor_msgs.CameraInfo),
            message_filters.Subscriber('pose', self.pose_type),
            message_filters.Subscriber('objects', motlee_msgs.ObjArray),
        ]
        if not self.image_view:
            self.subs = self.subs[3:] # don't subscribe to camera image if not viewing objects on image
        self.ts = message_filters.ApproximateTimeSynchronizer(self.subs, 100, 0.1)
        self.ts.registerCallback(self.cb)

        # ROS publishers
        self.pub_img = rospy.Publisher('object_viz/image_raw', sensor_msgs.Image, queue_size=1)
        self.pub_markers = rospy.Publisher('object_viz/markers', visualization_msgs.MarkerArray, queue_size=1)

    def cb(self, *msgs):
        """Detection callback - incorporates new measurements into map

        Args:
            dets_msg (motlee_msgs/ObjArray): landmark measurements and covariances in world frame
        """
        if self.image_view:
            msg_img, msg_cam_info, msg_pose, msg_objs = msgs
        else:
            msg_objs,  = msgs
            
        if self.image_view:
            img = self.bridge.compressed_imgmsg_to_cv2(msg_img, desired_encoding='bgr8')
            
            K = np.array(msg_cam_info.K).reshape((3,3))
            
            rect_color = (255, 0, 0)
            rect_thickness = 2
            
            for obj in msg_objs.objects:
                w = obj.width
                h = obj.height
                # point_b = transform(inv(pose_msg_2_T(msg_pose.pose)), np.array([obj.position.x, obj.position.y, obj.position.z]))
                point_c = transform(inv(self.T_BC) @ inv(pose_msg_2_T(msg_pose.pose)), np.array([obj.position.x, obj.position.y, obj.position.z]))
                point_c_ll = point_c + np.array([-w/2, h/2, 0]) # lower left
                point_c_ur = point_c + np.array([w/2, -h/2, 0]) # upper right
                if point_c_ll.item(2) > 0:
                    bbox_ll = xyz_2_pixel(point_c_ll, K).reshape(-1)
                    bbox_ur = xyz_2_pixel(point_c_ur, K).reshape(-1)
                    w = bbox_ur[0] - bbox_ll[0]
                    h = bbox_ll[1] - bbox_ur[1]
                    img = cv.rectangle(img, bbox_ll.astype(np.int32), 
                                    bbox_ur.astype(np.int32),
                                    color=rect_color,
                                    thickness=rect_thickness)
                    
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            img_msg.header = msg_objs.header
            self.pub_img.publish(img_msg)
        
        marker_arr = visualization_msgs.MarkerArray()
        for obj in msg_objs.objects:
            marker = visualization_msgs.Marker()
            # TODO: fix id
            marker.header = msg_objs.header
            marker.header.stamp = rospy.Time.now()
            marker.id = obj.id
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.scale.x = obj.width
            marker.scale.y = obj.width
            marker.scale.z = obj.height
            marker.lifetime = rospy.Duration.from_sec(1.0)
            marker.frame_locked = 1
            marker.color.a = 1
            marker.color.r = self.marker_color[0]
            marker.color.g = self.marker_color[1]
            marker.color.b = self.marker_color[2]
            marker.pose =  geometry_msgs.Pose()
            # Use robot camera frame for visualization
            # TODO: object map right now is reported to be in camera frame... this is not true, we should get the tf worked out so that
            # object_measurements are published from fastsam3d in the map frame or something
            # point_c = transform(inv(self.T_BC) @ inv(pose_msg_2_T(msg_pose.pose)), np.array([obj.position.x, obj.position.y, obj.position.z]))
            point = np.array([obj.position.x, obj.position.y, obj.position.z])
            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            if self.rdf:
                marker.pose.orientation.w = np.sqrt(2)/2
                marker.pose.orientation.x = np.sqrt(2)/2
            else:
                marker.pose.orientation.w = 1.0
                marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker_arr.markers.append(marker)

        self.pub_markers.publish(marker_arr)
        
        
if __name__ == "__main__":
    rospy.init_node("ojb_viz_node")

    ojb_viz_node = ObjVizNode()
    rospy.spin()