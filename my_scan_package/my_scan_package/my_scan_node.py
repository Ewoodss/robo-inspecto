import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from laser_geometry import LaserProjection




class MyScanNode(Node):
    def __init__(self):
        super().__init__('my_scan_node')
        # Subscribe to the scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        # Publisher for cloudpoint
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            'pointcloud',
            10
        )
        
        self.laser_projection = LaserProjection()
        
    def scan_callback(self, msg):
        # Process the laser scan data here
        # Access data using msg.field_name
        
        point_cloud = self.laser_projection.projectLaser(msg)

        point_cloud

        self.point_cloud_publisher.publish(point_cloud)

        

def main(args=None):
    print("Hello World!")
    rclpy.init(args=args)
    my_scan_node = MyScanNode()
    rclpy.spin(my_scan_node)
    my_scan_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
