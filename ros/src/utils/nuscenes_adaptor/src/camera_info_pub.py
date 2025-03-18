#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo

def camera_info_publisher():
    rospy.init_node('camera_info_publisher', anonymous=True)
    camera_info_pub = rospy.Publisher('/cam_front/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "camera_frame"

    # Intrinsic parameters (camera matrix)
    camera_info_msg.K = [
        1266.417203046554, 0.0, 816.2670197447984,
        0.0, 1266.417203046554, 491.50706579294757,
        0.0, 0.0, 1.0
    ]

    # Distortion parameters (set to zero if unknown or not needed)
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]

    # Rectification matrix (identity for no rectification)
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    # Projection matrix
    camera_info_msg.P = [
        1266.417203046554, 0.0, 816.2670197447984, 170.0,
        0.0, 1266.417203046554, 491.50706579294757, 1.0,
        0.0, 0.0, 1.0, 150.0
    ]

    # Dimensions (optional, set to your image size)
    camera_info_msg.width = 1920  # Replace with your image width
    camera_info_msg.height = 1080  # Replace with your image height

    while not rospy.is_shutdown():
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_pub.publish(camera_info_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        camera_info_publisher()
    except rospy.ROSInterruptException:
        pass

