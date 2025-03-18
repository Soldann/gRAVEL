#!/usr/bin/env python3

import rospy
import struct
from sensor_msgs.msg import PointCloud2, PointField
from nuscenes2bag.msg import RadarObject, RadarObjects
from std_msgs.msg import Header

# Define the callback for processing messages from radar_front
def radar_callback(msg):
    for radar_object in msg.objects:
        # Convert the incoming radar data to a PointCloud2 message
        point_cloud = convert_to_pointcloud2(radar_object, msg.header.stamp)
        # Publish the converted message
        pointcloud_pub.publish(point_cloud)

def convert_to_pointcloud2(data, timestamp):
    header = Header()
    header.stamp = timestamp
    header.frame_id = "radar_front"  # Replace with your desired frame

    # Define the PointCloud2 fields
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


    print(float(data.pose.x),
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
                int(data.vy_rms))

    # Create the point cloud data
    try:
        points = [
            struct.pack(
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
        ]
    except AttributeError as e:
        rospy.logerr(f"Attribute error in RadarObject fields: {e}")
        return None

    # Combine into a PointCloud2 message
    point_cloud = PointCloud2()
    point_cloud.header = header
    point_cloud.height = 1
    point_cloud.width = len(points)
    point_cloud.fields = fields
    point_cloud.is_bigendian = False
    point_cloud.point_step = 72  # Total size of a point in bytes
    point_cloud.row_step = point_cloud.point_step * point_cloud.width
    point_cloud.data = b"".join(points)
    point_cloud.is_dense = True

    return point_cloud


if __name__ == "__main__":
    rospy.init_node("radar_to_pointcloud2")

    # Create a publisher for the point cloud
    pointcloud_pub = rospy.Publisher("radar_front_pointcloud", PointCloud2, queue_size=10)

    # Create a subscriber to listen to the radar_front topic
    radar_sub = rospy.Subscriber("radar_front", RadarObjects, radar_callback)

    rospy.spin()
