#!/usr/bin/env python3

import rospy
import struct
from sensor_msgs.msg import PointCloud2, PointField
from nuscenes_adaptor.msg import RadarObject, RadarObjects
from std_msgs.msg import Header

def radar_callback(msg):
    point_cloud = convert_to_pointcloud2(msg.objects, msg.header.stamp)
    if point_cloud:
        pointcloud_pub.publish(point_cloud)

def convert_to_pointcloud2(data_list, timestamp):
    if not data_list:
        return None

    header = Header()
    header.stamp = timestamp
    header.frame_id = "radar_front"

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="dyn_prop", offset=12, datatype=PointField.UINT32, count=1),
        PointField(name="id", offset=16, datatype=PointField.UINT32, count=1),
        PointField(name="rcs", offset=20, datatype=PointField.FLOAT32, count=1),
        PointField(name="vx", offset=24, datatype=PointField.FLOAT32, count=1),
        PointField(name="vy", offset=28, datatype=PointField.FLOAT32, count=1),
        PointField(name="vx_comp", offset=32, datatype=PointField.FLOAT32, count=1),
        PointField(name="vy_comp", offset=36, datatype=PointField.FLOAT32, count=1),
        PointField(name="is_quality_valid", offset=40, datatype=PointField.UINT32, count=1),
        PointField(name="ambig_state", offset=44, datatype=PointField.UINT32, count=1),
        PointField(name="x_rms", offset=48, datatype=PointField.UINT32, count=1),
        PointField(name="y_rms", offset=52, datatype=PointField.UINT32, count=1),
        PointField(name="invalid_state", offset=56, datatype=PointField.UINT32, count=1),
        PointField(name="pdh0", offset=60, datatype=PointField.UINT32, count=1),
        PointField(name="vx_rms", offset=64, datatype=PointField.UINT32, count=1),
        PointField(name="vy_rms", offset=68, datatype=PointField.UINT32, count=1),
    ]

    points = []
    for data in data_list:
        try:
            point = struct.pack(
                'fffIIfffffIIIIIIII',
                float(data.pose.x),
                float(data.pose.y),
                float(data.pose.z),
                int(data.dyn_prop),
                int(data.id),
                float(data.rcs),
                float(data.vx),
                float(data.vy),
                float(data.vx_comp),
                float(data.vy_comp),
                int(data.is_quality_valid),
                int(data.ambig_state),
                int(data.x_rms),
                int(data.y_rms),
                int(data.invalid_state),
                int(data.pdh0),
                int(data.vx_rms),
                int(data.vy_rms),
            )
            points.append(point)
        except AttributeError as e:
            rospy.logerr(f"Attribute error in RadarObject fields: {e}")
            continue

    if not points:
        return None

    point_cloud = PointCloud2()
    point_cloud.header = header
    point_cloud.height = 1
    point_cloud.width = len(points)
    point_cloud.fields = fields
    point_cloud.is_bigendian = False
    point_cloud.point_step = 72  # Bytes per point
    point_cloud.row_step = point_cloud.point_step * point_cloud.width
    point_cloud.data = b"".join(points)
    point_cloud.is_dense = True

    return point_cloud

if __name__ == "__main__":
    rospy.init_node("radar_to_pointcloud2")
    pointcloud_pub = rospy.Publisher("radar_front_pointcloud", PointCloud2, queue_size=10)
    radar_sub = rospy.Subscriber("radar_front", RadarObjects, radar_callback)
    rospy.spin()