import numpy as np
from numpy.linalg import norm, inv
from scipy.spatial.transform import Rotation as Rot
import cv2 as cv
from copy import deepcopy

import rclpy
from rclpy.node import Node
import cv_bridge
import message_filters

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from motlee_msgs.msg import TrackStateArray

from .pose_handler import PoseHandler
from .utils import time_to_stamp, stamp_to_time, T_FLURDF, T_RDFFLU, \
    track_id_to_tuple
from motlee.utils.transform import transform, pos_quat_to_transform

class TrackViewerNode(Node):
    
    def __init__(self):
        super().__init__('track_viewer')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('track_time', .5),
                ('pose_from_odom', True),
                ('pose_topic', '/t265/odom/sample'),
                ('camera_prefix', 'd455/color')
            ]
        )
        
        self.track_time = self.get_parameter('track_time').value
        pose_from_odom = self.get_parameter('pose_from_odom').value
        pose_topic = self.get_parameter('pose_topic').value
        self.camera_prefix = self.get_parameter('camera_prefix').value
        
        # private vars
        self.tracks = dict()
        self.t = None
        self.brige = cv_bridge.CvBridge()

        # publishers / subscribers
        self.pub_trackimg = self.create_publisher(Image, 'tracks/image_raw', 1)     
        self.debug_sub = self.create_subscription(Image, 'detections/image_raw', self.debug_cb, 1)   
        
        self.pose_handler = PoseHandler(max_pose_len=100)
        pose_type = Odometry if pose_from_odom else PoseStamped
        self.sub_pose = self.create_subscription(pose_type, pose_topic, self.pose_handler.pose_cb, 10)

        subs = []
        subs.append(message_filters.Subscriber(self, CameraInfo, f'{self.camera_prefix}/camera_info'))
        subs.append(message_filters.Subscriber(self, CompressedImage, f'{self.camera_prefix}/image_raw/compressed'))
        subs.append(message_filters.Subscriber(self, TrackStateArray, f'tracks'))
        self.ts = message_filters.TimeSynchronizer(subs, 100)
        
        self.ts.registerCallback(self.tracks_cb)
    
    def tracks_cb(self, *msgs):
        print('getting something')
        cam_info = msgs[0]
        img_compressed = msgs[1]
        tracks = msgs[2]
        
        self.t = stamp_to_time(tracks.header.stamp)
        erase_before = self.t - self.track_time
        K = np.array(cam_info.k).reshape((3,3))
        ego_pose = self.pose_handler.get_pose_from_time(self.t)
        if ego_pose is None:
            return
        ego_pose[2] = 1.0 if 'd435' in self.camera_prefix else 1.5
        T_lb = pos_quat_to_transform(ego_pose[:3], ego_pose[3:])
        
        img = self.brige.compressed_imgmsg_to_cv2(img_compressed, desired_encoding='bgr8')

        # Add local coordinates to track state
        for track in tracks.tracks:
            pos_l = np.array([[track.position.x, track.position.y, track.position.z]]).T
            track_id_tuple = track_id_to_tuple(track.track_id)
            if track_id_tuple not in self.tracks:
                self.tracks[track_id_tuple] = [[], []]
            self.tracks[track_id_tuple][0].append(self.t)
            self.tracks[track_id_tuple][1].append(pos_l)

        to_delete = []
        for track_id in self.tracks:
            track = self.tracks[track_id]
            while len(track[0]) > 0 and track[0][0] < erase_before:
                del track[1][0]
                del track[0][0]
            if len(track[1]) == 0:
                to_delete.append(track_id)
        for track_id in to_delete:
            del self.tracks[track_id]

        for track_id in self.tracks:
            positions = self.tracks[track_id][1]
            pixels_iminus = None
            pixels_i = None
            for i in range(len(positions)):
                pos_l = positions[i]
                pos_b = transform(inv(T_lb), pos_l)
                pos_c = transform(T_RDFFLU, pos_b) # RDF camera coordinates of position
                # pos_c[1] = -pos_c[1] # TODO: why? something broken
                uvs = K @ pos_c
                uvs /= uvs.item(2)
                pixels_i = tuple(uvs.reshape(-1)[:2][:].astype(int).tolist())
                
                if pixels_iminus is not None:
                    cv.line(img, pixels_i, pixels_iminus, (0,255,0), thickness=4)
                if i == len(positions) - 1:
                    cv.putText(img, f'{track_id[1]}', pixels_i, cv.FONT_HERSHEY_PLAIN, 1 if 'd435' in self.camera_prefix else 2, (200,0,200), thickness=2)

                pixels_iminus = deepcopy(pixels_i)

        imgmsg = self.brige.cv2_to_imgmsg(img, encoding='bgr8')
        imgmsg.header = img_compressed.header
        self.pub_trackimg.publish(imgmsg)
        
        return
    
    def debug_cb(self, msg):
        # print(msg.)
        pass

def main(args=None):
    rclpy.init(args=args)
    track_viewer = TrackViewerNode()
    rclpy.spin(track_viewer)
    
    track_viewer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
