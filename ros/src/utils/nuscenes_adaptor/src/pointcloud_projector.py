#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, Image, CameraInfo
from sensor_msgs import point_cloud2
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped
import cv2
import numpy as np
import matplotlib.pyplot as plt
from cv_bridge import CvBridge
from pyquaternion import Quaternion

class PointCloudProjector:
    def __init__(self):
        rospy.init_node('pointcloud_projector')

        # Initialize subscribers
        self.pointcloud_sub = rospy.Subscriber('/lidar_top', PointCloud2, self.pointcloud_callback)
        self.image_sub = rospy.Subscriber('/cam_front/raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/cam_front/camera_info', CameraInfo, self.camera_info_callback)

        # Initialize publisher
        self.projected_pub = rospy.Publisher('/cam_front/projected', Image, queue_size=1)

        # Placeholder for data
        self.pointcloud = None
        self.image = None
        self.camera_matrix = None
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.spin()

    def pointcloud_callback(self, msg):
        self.pointcloud = msg
        # self.process()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process()

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        # self.process()

    def process(self):
        if self.pointcloud is None or self.image is None or self.camera_matrix is None:
            return

        # Extract points from PointCloud2
        points = self.extract_points(self.pointcloud)

        # Transform points from /lidar_top to /base_link to /cam_front
        transformed_points = self.transform_points(points, 'lidar_top', 'cam_front')
        # print(transformed_points)
        # Project points onto image
        projected_image = self.project_points_to_image(transformed_points, self.image, self.camera_matrix)

        # Publish the projected image
        projected_msg = self.bridge.cv2_to_imgmsg(projected_image, encoding='bgr8')
        self.projected_pub.publish(projected_msg)

    def extract_points(self, pointcloud_msg):
        # Extract points using the `read_points` utility
        points = point_cloud2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=False)
        points = np.array(list(points), dtype=np.float32).T  # Transpose to get shape (3, N)
        return points

    def transform_points(self, points, source_frame, target_frame):
        try:
            # Lookup transform from source_frame to target_frame
            transform = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

            # Convert transform to matrix
            translation = np.array([transform.transform.translation.x,
                                     transform.transform.translation.y,
                                     transform.transform.translation.z])
            rotation = transform.transform.rotation
            quaternion = Quaternion([rotation.w, rotation.x, rotation.y, rotation.z])
            rotation_matrix = quaternion.rotation_matrix

            # print(rotation_matrix)
            # print(translation)

            # rotation_matrix = np.array([[ 0.99997086, 0.00348797,  0.00679117],
            #                             [ 0.00672519, 0.01859214, -0.99980453],
            #                             [-0.00361355, 0.99982107,  0.01856814],])
            # translation = np.array([ 0.01190664, -0.32498627,-0.75900204])
            

            # Create transformation matrix
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
            transformation_matrix[:3, 3] = translation

            # Transform points
            points_homogeneous = np.vstack((points, np.ones((1, points.shape[1]))))  # Convert to homogeneous coordinates
            transformed_points = transformation_matrix @ points_homogeneous
            return transformed_points[:3, :]  # Convert back to 3D coordinates

        except Exception as e:
            rospy.logerr(f"Failed to transform points: {e}")
            return points

    def project_points_to_image(self, points, image, camera_matrix):
        # print(points)
        depths = points[2, :]
        min_dist = 1.0 # Distance from the camera below which points are discarded.

        # Project 3D points onto 2D image plane using camera matrix
        points_2d = camera_matrix @ points
        points_2d = points_2d / points_2d[2:3, :].repeat(3, 0)  # Normalize by z (depth)

        mask = np.ones(depths.shape[0], dtype=bool)
        mask = np.logical_and(mask, depths > min_dist)
        mask = np.logical_and(mask, points_2d[0, :] > 1)
        mask = np.logical_and(mask, points_2d[0, :] < image.shape[1] - 1)
        mask = np.logical_and(mask, points_2d[1, :] > 1)
        mask = np.logical_and(mask, points_2d[1, :] < image.shape[0] - 1)
        points_2d = points_2d[:, mask]

        # print(points_2d)

        # fig, ax = plt.subplots(figsize=(6, 6))
        # ax.imshow(image)
        # ax.scatter(points[0, :], points[1, :], c=depths, s=3)
        # ax.axis('off')
        # fig.canvas.draw()
        # image_from_plot = np.array(fig.canvas.renderer.buffer_rgba())
        # image = cv2.cvtColor(image_from_plot, cv2.COLOR_RGBA2BGR)
        # plt.close(fig)
        
        # Convert points to integers in one step to avoid repeated conversions
        points_2d_int = points_2d[:2, :].astype(int)

        # Use numpy to batch process instead of looping
        for x, y in points_2d_int.T:
            cv2.circle(image, (x, y), 3, (0, 255, 0), -1)



        # max_depth = 70  # Set min and max depth for colouring
        # min_depth = 0
        # start_colour = np.array([155, 0, 105])  # BGR
        # end_colour = np.array([100, 255, 0])

        # for i in range(points_2d.shape[1]):
        #     x, y = int(points_2d[0, i]), int(points_2d[1, i])
        #     intensity = (depths[i] - min_depth) / (max_depth - min_depth)  # Normalize depth to [0, 1]            x, y = int(points_2d[0, i]), int(points_2d[1, i])
        #     interpolated_colour = (1 - intensity) * start_colour + intensity * end_colour
        #     print(min_depth, max_depth, depths[i], intensity)
        #     cv2.circle(image, (x, y), 3, (int(interpolated_colour[0]), int(interpolated_colour[1]), int(interpolated_colour[2])), -1)

        return image

if __name__ == '__main__':
    PointCloudProjector()

