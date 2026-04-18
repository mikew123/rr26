#!/usr/bin/env python3


import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

from robo24_interfaces.msg import Barrels, Barrel

class rr26LidarCanNode(Node):
    # parameters?
        
    # dimensions of soda can
    canHeight = 0.125
    canRadius = 0.065/2

    maxBarrelDist = 0.8
    minBarrelDist = 0.15
    minJumpDist = 0.5

    def __init__(self):
        super().__init__('rr26_lidar_can_node')

        self.scan_msg_subscriber = self.create_subscription( LaserScan, 'scan', self.scan_msg_callback, 10)
        self.barrels_msg_publisher = self.create_publisher(Barrels, 'barrels', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"rr26LidarCanNode: Started")

    def cleanup(self) :
        self.barrels_msg_publisher.destroy()

    def barrelDet(self, msg:LaserScan) -> None :
        """
        The Lidar is also used for barrel racing, the camera blob detection is
        not used. 3 cans are to be detected, but only 2 will be detected when a 
        can blocks the can "behind" it

        A barrel (can) will look like a contiguous sequence of ranges with a variance
        of the radius of the can. The number of sequential ranges is determined by the
        diameter (width) of the can, based on the distance, minus a few on the ends where the can surface is
        perpedicular to the Lidar sensor

        """

        rmin = msg.range_min
        amin = msg.angle_min
        amax = msg.angle_max
        ainc = msg.angle_increment
        rays = msg.ranges
        nrays = len(rays)

        # self.get_logger().info(f"barrelDet: {rmin=} {amin=} {amax=} {ainc=} {nrays=}")

        brmsg = Barrels()

        barrelWidth = 2*self.canRadius

        dMax = 0
        dMin = 0 # Min distance in sequence
        iMin = 0 # index at min distance (barrel center)
        iCnt = 0 # count of rays in valid sequence
        d = rays[nrays-1] # last ray before 1st ray for diff
        dLast = d
        dDiff = self.minJumpDist
        maxNumCnt = 0

        detActive = False #set when a sequence start is detected with a jump and valid dist

        #TODO: manage barrel detection at angle = 0, sequence is divided between start and stop
        for i in range(nrays) :
            iCnt +=1

            dLast = d
            d = rays[i]
            dDiff = abs(d - dLast)

            dJmp = dDiff>=self.minJumpDist # jump in ray distance detected

            if detActive==False : # detection inactive wait for start jump
                if dJmp==True and d<=self.maxBarrelDist and d>self.minBarrelDist:
                    detActive = True
                    dMax = d
                    dMin = d
                    iMin = i
                    iCnt = 0
                    maxNumCnt = int(math.atan(barrelWidth/dMin)/ainc)
                    # self.get_logger().info(f"barrelDet: Start Jump A {i=} {d=} {maxNumCnt=}")

            else : # detection is active
                maxNumCnt = int(math.atan(barrelWidth/dMin)/ainc)
                iCntDiff = abs(iCnt-maxNumCnt)

                if dJmp==False : # get minimum dist at angle in sequence
                    if d>dMax :
                        dMax = d

                    if d<dMin :
                        dMin = d
                        iMin = i

                else : # possible end jump and/or start for sequential barrel detections
                    
                    diff = (dMax-dMin)
                    # self.get_logger().info(f"barrelDet: Jump det {maxNumCnt=} {iCnt=} {iMin=} {iCntDiff=} {dMin=} {dMax=}")

                    if diff<(barrelWidth/1) and iCntDiff<=3 : # end of valid sequence
                        detActive = False
                        a = (iMin * ainc) - math.pi # angle to barrel
                        # limit angle to +- pi
                        if a>math.pi :
                            a -= 2*math.pi
                        b = Barrel()
                        b.distance = dMin + barrelWidth/2 # distance to the center of barrel
                        b.angle = a
                        brmsg.barrel.append(b)
                        # self.get_logger().info(f"barrelDet: End jump {b=} {i=} {maxNumCnt=} {iCnt=} {iMin=} {dMin=} {diff=}")

                    if d<self.maxBarrelDist and d>self.minBarrelDist : # also start jump for next sequence
                        detActive = True
                        dMax = d
                        dMin = d
                        iMin = i
                        iCnt = 0
                        # self.get_logger().info(f"barrelDet: Start Jump B {i=} {d=} {maxNumCnt=}")
        # end of for loop

        # continue sequence processing if active
        i = -1

        while detActive==True :
            i+=1
            iCnt +=1

            dLast = d
            d = rays[i]
            dDiff = abs(d - dLast)

            # find end of sequence
            dJmp = dDiff>=self.minJumpDist # jump in ray distance detected if >= 1 meter
            if dJmp == False :
                if d>dMax :
                    dMax = d

                if d<dMin :
                    dMin = d
                    iMin = i

            else : # end of sequence jump detected
                maxNumCnt = int(math.atan(barrelWidth/dMin)/ainc)
                iCntDiff = abs(iCnt-maxNumCnt)
                diff = (dMax-dMin)
                # self.get_logger().info(f"barrelDet: Jump det B {maxNumCnt=} {iCnt=} {iMin=} {iCntDiff=} {dMin=} {dMax=}")

                if diff<(barrelWidth/1) and iCntDiff<=3 : # end of valid sequence
                    detActive = False
                    a = iMin * ainc # angle to barrel
                    # convert 0 to 2pi to +=pi
                    if a>math.pi :
                        a-= 2*math.pi
                    b = Barrel()
                    b.distance = dMin + barrelWidth/2 # distance to the center of barrel
                    b.angle = a
                    brmsg.barrel.append(b)
                    # self.get_logger().info(f"barrelDet: End jump {b=} {i=} {maxNumCnt=} {iCnt=} {iMin=} {dMin=} {diff=}")

                detActive=False # End jump detected

        self.barrels_msg_publisher.publish(brmsg)

        # if len(brmsg.barrel) > 0 :
        #     self.get_logger().info(f"barrelDet: {brmsg=}")


    # Lidar laser scan message callback
    def scan_msg_callback(self, msg:LaserScan) -> None :
        """
        Process the Lidar scan data to remove the rays that include
        the can that is being persued and create a scan_obs message
        which is used for Nav2 obstacle avoidance
        The idea is to not avoid approaching the can being persued

        The can is initialy identified using the OpenMV camera blob detection
        As the robot gets closer and the camera can not reliably ID it anymore 
        the Lidar data is used to track the can and pull it into the can
        catch basket. This replaces the single point L4 range detector sensor by
        Using the center rays for distance to object directly in front

        The Lidar is also used for barrel racing, the camera blob detection is
        not used. 3 cans are to be detected, but only 2 will be detected when a 
        can blocks the can "behind" it
        """

        # detect object in front for 6 can and provide distance.
        # this replaces the TOF L4 range sensor that has a narrow detect angle.


        # detect cans for barrel racing and provide distance and angle (not xy).
        # The center can can be up to 12 feet from the Lidar sensor at the start.
        # and the cans are in an equal triangle about 6 to 8 ft from each other.

        self.barrelDet(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = rr26LidarCanNode()
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

