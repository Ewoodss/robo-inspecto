import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class MyScanNode(Node):
    def __init__(self):
        super().__init__('my_scan_node')
        # Subscribe to the scan topic
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
    def scan_callback(self, msg):
        # Process the laser scan data here
        # Access data using msg.field_name
        pass

def main(args=None):
    print("Hello World!")
    rclpy.init(args=args)
    my_scan_node = MyScanNode()
    rclpy.spin(my_scan_node)
    my_scan_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
