#!/usr/bin/env python3


import rclpy

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class rr26ScanFixNode(Node):
    """
    Fix the rotational Lidar scan related distortions 
    """

    def __init__(self):
        super().__init__('rr26_scan_fix_node')

        self.cmd_vel_subscription = self.create_subscription(Twist, '/cmd_vel', 
                                                             self.cmd_vel_callback, 10)
        self.scan_msg_subscriber = self.create_subscription( LaserScan, 'scan', 
                                                            self.scan_msg_callback, 10)
        self.scan_fix_msg_publisher = self.create_publisher(LaserScan, 'scan_fix', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(f"rr26ScanFixNode: Started")

    def cleanup(self) :
        self.scan_fix_msg_publisher.destroy()


    # robot velocities used for motion compensation of Lidar scan data
    linX:float = 0.0
    angZ:float = 0.0

    def timer_callback(self) -> None :
        """
        Clears the current velocities when there is no /cmd_vel data
        """
        linX = 0.0
        angZ = 0.0


    def cmd_vel_callback(self, msg:Twist) -> None :
        """
        Extracts the current linear and angular velocities
        which are used to "fix" the Lidar scan data
        The velocities are saved in global variables 
        The saved velocities are cleared to 0.0 if the timer times out
        """

        self.timer.reset()

        self.linX = msg.linear.x
        self.angZ = msg.angular.z
    
    def scan_msg_callback(self, msg:LaserScan) -> None :
        """
        Processes the raw Lidar scan data to fix the rotational
        and directional distortions caused my robot movement while 
        the Lidar is scanning (rotating)
        A corrected /scan_fix message is published
        """

        scanLag:float = 0.0

        linx = self.linX
        angZ = self.angZ

        # copy all the original scan message variables before fixing
        msg_fix = msg
        
        # extract the lidar scan parameters
        rmin = msg.range_min
        amin = msg.angle_min
        amax = msg.angle_max
        ainc = msg.angle_increment
        rays = msg.ranges
        nrays = len(rays)
        scanT = msg.scan_time

        # calc scan rotation motion compensation
        angErr = angZ * scanT
        # amax += angErr
        amin += angErr
        ainc = (amax - amin) / (nrays -1)

        # calc scan lag motion compensation
        alagErr = angZ * scanLag
        amin += alagErr
        amax += alagErr

        # Update scan data in message
        # msg.angle_max = amax
        # msg.angle_min = amin
        # msg.angle_increment = ainc

        self.scan_fix_msg_publisher.publish(msg_fix)

        if angZ != 0.0 :
            self.get_logger().info(f"scan_msg_callback: {angZ=} {scanT=} {angErr=} {ainc=} {amin=} {amax=} {nrays=}")

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

