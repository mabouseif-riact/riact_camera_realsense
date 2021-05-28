import rclpy
from rclpy.node import Node

import std_msgs
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, PointCloud2, PointField
import sensor_msgs
# import sensor_msgs.point_cloud2 as pcl2
from .realsense import RealSenseCamera
import ctypes
import math
import struct
import time
from .point_cloud2 import *
import os
import glob

class PointcloudPublisher(Node):

    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.cam = RealSenseCamera(1)
        self.publisher_ = self.create_publisher(PointCloud2, 'image_topic', 1)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.fields = [PointField(name='x', offset=0,
                         datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4,
                         datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8,
                         datatype=PointField.FLOAT32, count=1)]


    def timer_callback(self):
        
        color_image, depth_image, points = self.cam.get_frame()
        start = time.time()
        # msg = self.verts_to_cloud2(points, depth_image)
        msg = create_cloud_xyz32(Header(frame_id='frame'), points)
        elapsed = time.time() - start
        print("Elapsed looping points: {}".format(elapsed))
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"')#  % msg.data)
        self.i += 1

    # def verts_to_cloud2(self, points, depth_image):
        # cloud_struct = struct.Struct(_get_struct_fmt(False, self.fields))
        # buff = ctypes.create_string_buffer(cloud_struct.size * len(points))
        # point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
        # offset = 0
        # start = time.time()
        # for p in points:
        #     pack_into(buff, offset, *p)
        #     offset += point_step
        # elapsed = time.time() - start
        # print("Elapsed looping points: {}".format(elapsed))

        # cloud2 = PointCloud2(header=Header(frame_id='frame'),
        #                height=depth_image.shape[0], # 1,
        #                width=depth_image.shape[1], # len(points),
        #                is_dense=False,
        #                is_bigendian=False,
        #                fields=self.fields,
        #                point_step=cloud_struct.size, # 12
        #                row_step=cloud_struct.size * depth_image.shape[1],  # cloud_struct.size * len(points),
        #                data=buff.raw)
        


        # # cloud2 = PointCloud2()
        # # cloud2.fields = fields
        # # cloud2.height = depth_image.shape[0]
        # # cloud2.width = depth_image.shape[1]
        # # cloud2.point_step = 4   # Length of a point in bytes
        # # cloud2.row_step= cloud2.point_step * cloud2.width     # Length of a row in bytes
        # # # uint8[] cloud2.data = ?         # Actual point data, size is (row_step*height)
        # # # bool is_dense 
        # # print(cloud2)

           


        # return cloud2

def main(args=None):
    rclpy.init(args=args)

    pointcloud_publisher = PointcloudPublisher()

    rclpy.spin(pointcloud_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pointcloud_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()