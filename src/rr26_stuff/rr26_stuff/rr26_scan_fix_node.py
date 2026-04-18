#!/usr/bin/env python3


import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan

class rr26ScanFixNode(Node):
    """
    Fix the rotational Lidar scan related distortions 
    """

    def __init__(self):
        super().__init__('rr26_scan_fix_node')

        self.scan_msg_subscriber = self.create_subscription( LaserScan, 'scan', 
                                                            self.scan_msg_callback, 10)
        self.scan_fix_msg_publisher = self.create_publisher(LaserScan, 'scan_fix', 10)

        self.get_logger().info(f"rr26ScanFixNode: Started")

    def cleanup(self) :
        self.scan_fix_msg_publisher.destroy()

    def scan_msg_callback(self, msg:LaserScan) -> None :
        """
        Processes the raw Lidar scan data to fix the rotational
        and directional distortions caused my robot movement while 
        the Lidar is scanning (rotating)
        A corrected /scan_fix message is published
        """

        # copy all the original scan message variables before fixing
        msg_fix = msg
        # TODO: fix distortions
        
        self.scan_fix_msg_publisher.publish(msg_fix)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = rr26ScanFixNode()
        rclpy.spin(node, MultiThreadedExecutor())  # Will exit on Ctrl+C
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        node.cleanup()
    finally:
        if node is not None:
            node.destroy_node()
        # rclpy.shutdown() # shutdown is called in "context" no need to call again

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

