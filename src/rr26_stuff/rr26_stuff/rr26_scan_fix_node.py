#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from builtin_interfaces.msg import Time, Duration

class rr26ScanFixNode(Node):
    """
    Fix the rotational Lidar scan related distortions 
    """

    def __init__(self):
        super().__init__('rr26_scan_fix_node')

        self.wheel_odom_subscription = self.create_subscription(Odometry, 'wheel_odom', 
                                                             self.wheel_odom_callback, 10)
        self.scan_msg_subscriber = self.create_subscription(LaserScan, 'scan', 
                                                            self.scan_msg_callback, 10)
        self.scan_fix_msg_publisher = self.create_publisher(LaserScan, 'scan_fix', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(f"rr26ScanFixNode: Started")

    def cleanup(self):
        self.scan_fix_msg_publisher.destroy()

    # robot velocities used for motion compensation of Lidar scan data
    odomPause: bool = False
    odomTstamp: Time = None
    linVelX: float = 0.0
    angVelZ: float = 0.0
    odomTstamp_last: Time = None
    linVelX_last: float = 0.0
    angVelZ_last: float = 0.0
    linAccX: float = 0.0
    angAccZ: float = 0.0
    linCurve: list = []

    def timer_callback(self) -> None :
        """
        Clears the current velocities when there is no /cmd_vel data
        """
        self.odomTstamp = None
        self.odomTstamp_last = None
        self.linVelX = 0.0
        self.angVelZ = 0.0
        self.linVelX_last = 0.0
        self.angVelZ_last = 0.0
        self.linAccX = 0.0
        self.angAccZ = 0.0

    def wheel_odom_callback(self, msg: Odometry) -> None:
        """
        Save wheel odometry data in global variables for 
        Lidar motion compensation
        The wheel odom rate is higher than the Lidar scan rate
        """
        self.timer.reset()

        if self.odomPause:
            return
        # update odom encoder data when not paused
        # the application pauses this while accessing the data

        self.odomTstamp_last = self.odomTstamp
        self.linVelX_last = self.linVelX
        self.angVelZ_last = self.angVelZ

        self.odomTstamp = msg.header.stamp
        self.linVelX = msg.twist.twist.linear.x
        self.angVelZ = msg.twist.twist.angular.z

        # calc acceleration
        if self.odomTstamp != None and self.odomTstamp_last != None:
            t0 = self.odomTstamp_last.sec + 1e-9 * self.odomTstamp_last.nanosec
            t1 = self.odomTstamp.sec + 1e-9 * self.odomTstamp.nanosec
            dt = t1 - t0
            if dt > 0 and dt < 1:
                self.linAccX = (self.linVelX - self.linVelX_last) / dt
                self.angAccZ = (self.angVelZ - self.angVelZ_last) / dt
            else:
                self.linAccX = 0.0
                self.angAccZ = 0.0
    
    def scan_msg_callback(self, msg: LaserScan) -> None:
        """
        The Lidar device which created the scan data is a RPLidar C1
        Processes the raw Lidar scan data to fix the rotational
        and directional distortions caused by robot movement while 
        the Lidar is scanning (rotating)
        A corrected /scan_fix message is published
        """

        # stop odom data from being updated while accessing it
        self.odomPause = True
        odomTstamp: Time = self.odomTstamp
        linX: float = self.linVelX
        angZ: float = self.angVelZ
        linAccX: float = self.linAccX
        angAccZ: float = self.angAccZ
        self.odomPause = False

        odomTsec: float = None
        if odomTstamp != None:
            odomTsec = odomTstamp.sec + 1e-9 * odomTstamp.nanosec

        # extract the lidar scan parameters
        rmin = msg.range_min
        amin = msg.angle_min
        amax = msg.angle_max
        ainc = msg.angle_increment
        ranges = msg.ranges
        nranges = len(ranges)
        scanT = msg.scan_time

        scanTstamp: Time = msg.header.stamp
        scanTsec: float = scanTstamp.sec + 1e-9 * scanTstamp.nanosec  # Start scan time
        scanTsec_end: float = scanTsec + scanT  # End scan time

        # Extrapolate the odom rotational velocity to the middle of the scan time
        lagTsec: float = 0.0
        if odomTsec != None:
            lagTsec = (scanTsec_end - odomTsec)  # Time from odom measurement to scan end
            # Apply lag compensation: add the rotational velocity change over the lag period
            angZ += angAccZ * lagTsec

        # calc distance compensation curve once
        # A cos function is mpy with a linear functio 1.0 to 0.0
        # Distances at start of scan are adjusted a lot
        # Distances at end of scan are not adjusted as much
        # This causes the range values to be ajusted for the end time of the sweep
        # TODO: calc linear velocity for each range value using linear acceleration
        #       this could improve motion correction
        if self.linCurve == [] :
            for i in range(0, nranges) :
                line = 1.0 - float(i)/nranges # 1 to 0
                dadj = line * math.cos(line*2*math.pi)
                self.linCurve.append(dadj)
            # self.get_logger().info(f"scan_msg_callback: {self.linCurve=}")
        linCurve:list = self.linCurve

        # calc scan motion compensation

        # angular rotation
        # While robot rotates the scan angle range increases or decreases
        angErr = angZ * scanT
        amax -= angErr
        # amin -= angErr 
        ainc = (amax - amin) / (nranges - 1) if nranges > 1 else 0.0

        # linear distance
        # distance error caused by movement for 1 scan time
        distErr = linX * scanT
        for i in range(0, nranges) :
            ranges[i] += distErr * (1-linCurve[i]) # reversed compensation curve

        # calc scan lag motion compensation
        #NOTE: does not seem to help + or - adjust
        # alagErr = angZ * lagTsec
        # amin -= alagErr
        # amax -= alagErr

        # Update scan data in message
        
        # set fixed scan timestamp to end of scan
        ns: int = msg.header.stamp.nanosec + int(1e9 * scanT)
        sec: int = msg.header.stamp.sec
        if ns >= 1000000000:
            # handle nsec overflow
            sec += 1
            ns -= 1000000000
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = ns

        msg.angle_max = amax
        msg.angle_min = amin
        msg.angle_increment = ainc
        msg.ranges = ranges

        self.scan_fix_msg_publisher.publish(msg)

        if angZ != 0.0:
            self.get_logger().info(f"scan_msg_callback: {lagTsec=:.6f} {linX=:.3f} {angZ=:.3f} {scanT=:.3f} {distErr=:.3f} {angErr=:.3f} {ainc=:.6f} {amin=:.3f} {amax=:.3f} {nranges=}")


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

