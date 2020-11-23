
import os
import os.path as osp
import json
import time
import argparse
import numpy as np
import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
import sensor_msgs.point_cloud2 as pc2

from utils import *
# from img_utils import filter_depth, create_gif

class ImageCollector:


    def __init__(self, data_path, reset=False):
        rospy.init_node('pose_reader')

        self.data_path = data_path
        self.imgfolder = osp.join(self.data_path, 'images')
        self.img_paths_file = osp.join(self.data_path, 'img_paths_file.txt')
        try:
            os.mkdir(self.data_path)
        except Exception as e:
            pass
        try:
            os.mkdir(self.imgfolder)
        except Exception as e:
            pass

        topic = '/camera/depth/camera_info'
        rospy.Subscriber(topic, CameraInfo, self.get_depth_cam_info)
        topic = '/orb_slam2_mono/map_points'
        rospy.Subscriber(topic, PointCloud2, self.get_map_pts)
        topic = '/orb_slam2_mono/pose'
        rospy.Subscriber(topic, PoseStamped, self.get_pose)
        topic = '/octomap_point_cloud_centers'
        rospy.Subscriber(topic, PointCloud2, self.get_octomap)

        self.numpy_path = osp.join(self.data_path, 'pose.npy')
        self.image_paths = []
        self.mapts = None
        self.pose = None
        if osp.exists(self.numpy_path):
            self.poses = np.load(self.numpy_path).tolist()
        else:
            self.poses = []
        self.depth_cam_info = None

        self.save_depth_cam_info()

    def save_depth_cam_info(self):
        self.depth_cam_info = None
        while self.depth_cam_info is None:
            rospy.sleep(0.1)
        cam_info = {}
        cam_info['img_width'] = self.depth_cam_info.width
        cam_info['img_height'] = self.depth_cam_info.height
        cam_info['fx'] = self.depth_cam_info.K[0]
        cam_info['x_offset'] = self.depth_cam_info.K[2]
        cam_info['fy'] = self.depth_cam_info.K[4]
        cam_info['y_offset'] = self.depth_cam_info.K[5]
        self.cam_info = cam_info
        with open(self.data_path + '/camera_params.json', 'w') as outfile:
            json.dump(cam_info, outfile)

    def get_depth_cam_info(self, cam_info):
        self.depth_cam_info = None
        try:
            self.depth_cam_info = cam_info
        except CameraInfoMissingError as e:
            rospy.logerr(e)

    def get_map_pts(self, mapts):
        self.mapts = None
        try:
            self.mapts = mapts
        except Exception as e:
            rospy.logerr(e)

    def get_pose(self, pose):
        self.pose = None
        try:
            self.pose = pose
        except Exception as e:
            print('exception:', e)
            rospy.logerr(e)

    def get_octomap(self, octomap):
        self.octomap = None
        try:
            self.octomap = octomap
        except Exception as e:
            rospy.logerr(e)

    def save(self):
        if self.mapts is not None:

            mapts = get_xyz_points(pointcloud2_to_array(self.mapts))
            print('pts:', len(mapts))
            numpy_path = osp.join(self.data_path, 'mapts.npy')
            np.save(numpy_path, mapts)

        if self.octomap is not None:

            octomap = get_xyz_points(pointcloud2_to_array(self.octomap))
            print('octo:', len(octomap))
            numpy_path = osp.join(self.data_path, 'octomap.npy')
            np.save(numpy_path, octomap)

        if self.pose is not None:

            pose = self.pose
            pose = np.array([
              pose.pose.position.x,
              pose.pose.position.y,
              pose.pose.position.z,
              pose.pose.orientation.x,
              pose.pose.orientation.y,
              pose.pose.orientation.z,
              pose.pose.orientation.w
            ])
            print('pose:', pose)
            self.poses.append(pose)
            np.save(self.numpy_path, np.array(self.poses))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
      "svdir", nargs='?', type=str,
      default="/home/iclee141/workspace/motion_plan/motion_planning/slam_plan/slam_data",
      help="path to save directory"
    )
    # parser.add_argument(
    #   "-t", "--time",        type=int, default=5, help="recording time"
    # )
    # parser.add_argument(
    #   "-fps", "--framerate", type=int, default=3, help="frame rate"
    # )
    # parser.add_argument(
    #   "-r", "--reset", default=False, const=True,
    #   help="whether to overwrite any currently saved data in the designated folder"
    # )
    config = parser.parse_args()

    data_path = config.svdir
    try:
        os.mkdir(data_path)
    except Exception as e:
        # print(e)
        pass
    print('save path: {}'.format(data_path))

    img_col = ImageCollector(data_path)

    time.sleep(0.15)
    img_col.save()


if __name__ == '__main__':
    main()
