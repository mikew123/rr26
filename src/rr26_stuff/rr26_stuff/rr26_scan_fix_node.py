#!/usr/bin/env python3

from copy import deepcopy

import rclpy
import math

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range

from nav_msgs.msg import Odometry

from builtin_interfaces.msg import Time

class rr26ScanFixNode(Node):
    """
    Fix the rotational Lidar scan related distortions and /scan_fix is published
    The front range is determined and /front_range is published
    The width of a can is removed to publish /scan_nocan for obstical detection
    to not consider front can an obstacle
    """

    # dimensions of soda can
    canHeight   = 0.125
    canDiameter = 0.065
    canRadius   = canDiameter/2

    def __init__(self):
        super().__init__('rr26_scan_fix_node')

        self.wheel_odom_subscription = self.create_subscription(Odometry, 'wheel_odom', 
                                                             self.wheel_odom_callback, 10)
        self.scan_msg_subscriber = self.create_subscription(LaserScan, 'scan', 
                                                            self.scan_msg_callback, 10)
        
        self.scan_fix_msg_publisher = self.create_publisher(LaserScan, 'scan_fix', 10)
        self.scan_nocan_msg_publisher = self.create_publisher(LaserScan, 'scan_nocan', 10)
        self.front_range_msg_publisher = self.create_publisher(Range, 'front_range', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info(f"rr26ScanFixNode: Started")

    def cleanup(self):
        self.scan_fix_msg_publisher.destroy()
        self.scan_nocan_msg_publisher.destroy()
        self.front_range_msg_publisher.destroy()

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
        Used for Lidar motion compensation
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


    def scan_msg_callback(self, msg:LaserScan) -> None:
        """
        The Lidar device which created the scan data is a RPLidar C1
        Publishes motion compensated scan data
        Publishes a scan message with a front can removed
        Publishes a range message distance to what is directly in front
        """

        # motion compensate the scan data and return the fixed scan message
        scan_fix:LaserScan = self.scan_fix(msg)

        # determine the range distance of what is directly in front
        front_range:float = self.front_range(scan_fix)

        # remove the scan data for a can in front of the robot
        self.scan_nocan(scan_fix, front_range)


    def scan_fix(self, msg: LaserScan) -> LaserScan :
        """
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
        scanTsec: float = scanTstamp.sec + (1e-9 * scanTstamp.nanosec) # float seconds
        scanTsec_end: float = scanTsec + scanT  # End scan time

        # Extrapolate the odom velocity
        lagTsec: float = 0.0
        if odomTsec != None:
            lagTsec = (scanTsec_end - odomTsec)  # Time from odom measurement to scan end
            # Apply lag compensation: add the rotational velocity change over the lag period
            angZ += angAccZ * (lagTsec * 1) # adjust to middle of scan time for average vel
            linX += linAccX * lagTsec

        # calc distance compensation curve once
        # A cos function is mpy with a linear functio 1.0 to 0.0
        # Distances at start of scan are adjusted a lot
        # Distances at end of scan are not adjusted as much
        # This causes the range values to be ajusted for the end time of the sweep
        # TODO: calc linear velocity for each range value using linear acceleration
        #       this could improve motion correction
        if self.linCurve == [] :
            for i in range(0, nranges) :
                line = float(i)/nranges # 0 to 1
                dadj = math.fabs(line * math.cos(line*2*math.pi))
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
        # estimated distance error per range
        # distErrdt = (linAccX * (scanT/nranges)) * scanT
        for i in range(0, nranges) :
            # adjust linear velocity over time for better motion compensation
            # distErr += distErrdt
            ranges[i] += distErr * linCurve[i]

        # Update scan data in message
        
        # set fixed scan timestamp to the end of scan
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

        # if angZ != 0.0:
        #     self.get_logger().info(f"scan_msg_callback: {lagTsec=:.6f} {linX=:.3f} {angZ=:.3f} {scanT=:.3f} {distErr=:.3f} {angErr=:.3f} {ainc=:.6f} {amin=:.3f} {amax=:.3f} {nranges=}")

        return(deepcopy(msg))

    def front_range(self, scan_fix:LaserScan) -> Range :
        """
        Publish /front_range using the Lidar scan data
        The detected range distance is the minimum of the 
        scan distances in the field of view around 0 degrees
        which is in the front of the robot
        The range distance is relative to the Lidar scan sensor location
        Returns the detected range, Inf=invalid
        """
        
        # the range distance is relative to the Lidar scanner

        # the fov of the range is similar to standard OpenMV lens
        range_fov:float = math.radians(70.0)
        range_min:float = 0.100 # 100 mm
        range_max:float = 2.000 # 2 meters

        # extract scan parameters
        rmin = scan_fix.range_min
        amin = scan_fix.angle_min
        amax = scan_fix.angle_max
        ainc = scan_fix.angle_increment
        ranges = scan_fix.ranges
        nranges = len(ranges)
        header = scan_fix.header

        front_idx = int(nranges/2)
        range = ranges[front_idx]

        # determine the range of an object in the field of view
        # Select the 3 minimum distances and take the middle value
        mid = nranges/2
        fovIdxCnt = int(range_fov/ainc)
        fovMinIdx = int(mid - fovIdxCnt/2)
        fovMaxIdx = int(mid + fovIdxCnt/2)
        fovRanges = ranges[fovMinIdx:fovMaxIdx]

        # select 3 minimums
        fovMinRanges = [math.inf, math.inf, math.inf]
        for i in [0,1,2] :
            min = math.inf
            minIdx = None
            idx = 0
            for r in fovRanges :
                if r < min :
                    min = r
                    minIdx = idx
                idx +=1
            if minIdx != None :
                fovMinRanges[i] = min
                fovRanges[minIdx] = math.inf

        # select the middle min range to filter out extremes (brute force)
        range = None
        if fovMinRanges[0] < fovMinRanges[1] :
            if fovMinRanges[1] < fovMinRanges[2] :
                range = fovMinRanges[1]
            else :
                range = fovMinRanges[2]
        elif fovMinRanges[1] < fovMinRanges[2] :
            if fovMinRanges[0] < fovMinRanges[2] :
                range = fovMinRanges[0]
            else :
                range = fovMinRanges[2]
        else :
            if fovMinRanges[0] < fovMinRanges[1] :
                range = fovMinRanges[0]
            else :
                range = fovMinRanges[1]

        # limit range distances
        if range>range_max or range<range_min : range = math.inf

        front_range = Range()
        front_range.header= header
        front_range.range = range
        front_range.field_of_view = range_fov
        front_range.min_range = range_min
        front_range.max_range = range_max

        self.front_range_msg_publisher.publish(front_range)

        # self.get_logger().info(f"{range=} {fovMinRanges=} {fovRanges=}")

        return(deepcopy(front_range))

    def scan_nocan(self, scan_fix:LaserScan, front_range:Range) -> None :
        """
        Publish /scan_nocan by removing any can in front of the robot
        using the motion compensated scan data and the detected range distance
        If a can is qulified at the given distance then the scan data is removed
        for the width of the can
        This is used for obstical detetction and ignores the can it is trying to get
        The scan data is removed by setting the range value to Inf
        """

        # extract range parameters
        frontRangeFov:float = front_range.field_of_view
        canRange:float      = front_range.range

        # set the can distance parameters about the range distance
        canRangeMin:float   = canRange-self.canRadius
        canRangeMax:float   = canRange+(1.2*self.canRadius)
        
        # extract scan parameters
        ainc = scan_fix.angle_increment
        ranges = scan_fix.ranges
        nranges = len(ranges)

        # TODO? compute actual 0 deg middle for scan range ?
        scanRangeCnt = frontRangeFov/ainc
        scanRangeMin = int(nranges/2 - scanRangeCnt/2) -1
        scanRangeMax = int(nranges/2 + scanRangeCnt/2) +1
        
        # Qualify can before invalidating some scan data so the can is not in the scan
        # The amount of scan data invalidated is a bit more than the width of the can
        # The closer the can the more data is invalidated
        # Increase to ensure no can is detected

        canScanAngle = 2*math.atan2(self.canRadius, canRange)
        canScanPoints = int(canScanAngle/ainc)

        # detect can begining and end points in scan range data
        canBeg = None
        canEnd = None
        canWid = None

        # find sequence of scan data within rage value +- the can radius
        for i in range(scanRangeMin, scanRangeMax+1) :
            d = ranges[i]
            if canBeg==None :
                if d!=math.inf and (d>canRangeMin and d<canRangeMax) :
                    canBeg = i-1
            elif canEnd==None :
                if d!=math.inf and (d<canRangeMin or d>canRangeMax) :
                    canEnd = i+1
                    break

        if canBeg!=None and canEnd!=None :
            canWid = canEnd - canBeg +1
        
        # validate can width
        if canWid!=None :
            if abs(canWid-canScanPoints)>(canScanPoints/10.0 +7.0) : #5.0
                canWid = None

        # alter the motion compensated Lidar scan data to remove the can in FOV
        scan_nocan:LaserScan = deepcopy(scan_fix)

        if canWid!=None :
            for i in range(canBeg-4, canEnd+5) :
                scan_nocan.ranges[i] = math.inf

        self.scan_nocan_msg_publisher.publish(scan_nocan)

        # self.get_logger().info(f"{canRange=:0.3f} {canRangeMin=} {canRangeMax=} {scanRangeMin=} {scanRangeMax=} {canBeg=} {canEnd=} {canWid=} {canScanAngle=:0.3f} {canScanPoints=} {scanRangeMin=} {scanRangeMax=}")


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

