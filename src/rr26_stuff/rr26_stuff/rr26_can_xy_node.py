#!/usr/bin/env python3

# 4/18/2025 MRW copied from robo24_can_xy_node.property
# 2/5/2026 MRW converted to LifecycleNode
# 3/25/2026 converted back to non-lifecycle

import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

from robo24_interfaces.msg import Barrels, Barrel

class Robo24CanXYNode(Node):
    # parameters?
        
    # dimensions of soda can
    canHeight = 0.125
    canRadius = 0.065/2

    #SVGA image is 800x600
    imgRngX = 800 #Pixels
    imgRngY = 600 #Pixels
    #Camera
    HFOV = 70.8 #Degrees horizontal left-right
    VFOV = 55.6 #Degrees vertical up-down
    camThetaOffsetY = -0.0 #Degrees offset from vertical level

    heightTol = 70 #50 # Tolerance for height tolerance in percent

    # TOF array
    tofXY = np.zeros([8,24], dtype=int)

    # median filter data arrays: must be odd sized array
    medianFilterDataX = [0.0,0.0,0.0,0.0,0.0]
    medianFilterDataY = [0.0,0.0,0.0,0.0,0.0]
    medianFilterDataT = [0.0,0.0,0.0,0.0,0.0]

    maxBarrelDist = 0.8
    minBarrelDist = 0.2
    minJumpDist = 1.0

    def __init__(self):
        super().__init__('robo24_can_xy_node')

        self.openmv_msg_subscriber = self.create_subscription( String, 'openmv_msg', self.openmv_msg_callback, 10)
        self.tof8x8x3_msg_subscriber = self.create_subscription( String, 'tof8x8x3_msg', self.tof8x8x3_msg_callback, 10)
        self.scan_msg_subscriber = self.create_subscription( LaserScan, 'scan', self.scan_msg_callback, 10)
        self.tofxydebug_msg_publisher = self.create_publisher(String, 'tofxydebug_msg', 10)
        self.blobxydebug_msg_publisher = self.create_publisher(String, 'blobxydebug_msg', 10)
        self.scan_obs_msg_publisher = self.create_publisher(LaserScan, 'scan_obs', 10)
        self.barrels_msg_publisher = self.create_publisher(Barrels, 'barrels', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Robo24CanXYNode: Started")

    def cleanup(self) :
        self.tofxydebug_msg_publisher.destroy()
        self.blobxydebug_msg_publisher.destroy()
        self.scan_obs_msg_publisher.destroy()
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

            dJmp = dDiff>=self.minJumpDist # jump in ray distance detected if >= 1 meter

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

                    if diff<(barrelWidth/2) and iCntDiff<=2 : # end of valid sequence
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

                if diff<(barrelWidth/2) and iCntDiff<=2 : # end of valid sequence
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


        # detect up to 3 cans for barrel racing and provide distance and angle (not xy).
        # The center can can be up to 12 feet from the Lidar sensor at the start.
        # and the cans are in an equal triangle about 6 to 8 ft from each other.

        self.barrelDet(msg)



    # called when openmv detects a can blob
    # create a dynamic object XY that can be used to drive robo24
    def openmv_msg_callback(self, msg) :
        #self.get_logger().info(f"OpenMV {msg=}")
        #parse message
        strArray = msg.data.split(" ")
        if strArray[0]!="SVGA" or len(strArray)!=7:
            self.get_logger().error(f"Openmv message format error: {msg.data}")
            return
    
        try:
            blobX = int(strArray[1]) #X location in image
            blobY = int(strArray[2]) #Y location in image
            blobA = int(strArray[3]) #Area of object
            blobH = int(strArray[4]) #Height of object
            blobR = float(strArray[5]) #Detection  rate
            blobT = int(strArray[6])   #Threshold num used 
        except:
            self.get_logger().error(f"Openmv message parse error: {msg.data}")
            return

        # use blob area to qualify blob based on distance
        # convert blobY to +- distance from center
        # blobX is already +- distance from center
        blobY = int(blobY - self.imgRngY/2)
        if(blobY<0) :
            # get map XY coordinates from blob coordinates using triangulation 
            # values in mm and degrees
            (X,Y,thetaX) = self.mapXYFromBlobXH(blobX, blobH)

            #self.get_logger().info(f"BLOB {blobX = } {blobH = } {thetaX = } {X = } {Y = }")

            # make sure distance (X) is positive and > 0
            if X > 0 :
                # publish a can transform from blobXY conversion
                # Convert to Meters and Radians
                X_m = X/1000.0
                Y_m = Y/1000.0
                thetaX_r = thetaX/180.0 * math.pi

                # filter XYT with a median filter

                (self.medianFilterDataX, X_mFiltered) = medianFilter(self.medianFilterDataX, X_m)
                (self.medianFilterDataY, Y_mFiltered) = medianFilter(self.medianFilterDataY, Y_m)
                (self.medianFilterDataT, thetaX_r_mFiltered) = medianFilter(self.medianFilterDataT, thetaX_r)
                
                #self.get_logger().info(f"{X_mFiltered=} {Y_mFiltered=} {thetaX_r_mFiltered=}")
                                       
                # make sure X filtered is not zero for division
                if X_mFiltered > 0.0 :
                    # use blob area to qualify blob based on distance before creating TF
                    # NOTE: 50 is area at 1.0 meters; area is 1/100 from openmv
                    # use blob height to qualify blob based on distance
                    # NOTE: 95 is the height 
                    blobHMax = (1+(self.heightTol/100))*(95/X_mFiltered)
                    blobHMin = (1-(self.heightTol/100))*(95/X_mFiltered)
                    if (blobH<=blobHMax and blobH>=blobHMin) :
                        # TODO: Why do I need to negate Y?
                        self.broadcast_tf("cam_link","can",(X_mFiltered, -Y_mFiltered, thetaX_r_mFiltered))
                        # self.broadcast_tf("base_link","can",(X_mFiltered, -Y_mFiltered, thetaX_r_mFiltered))
                        # publish a debug message
                        strMsg = f"A{blobA} H{blobH} XY {(blobX,blobY)}  ({X_mFiltered: .3f},{Y_mFiltered: .3f}) Tr{thetaX_r_mFiltered: .3f}" # TOF {(tofX,tofY)} {tofDist}"
                        emsg = String()
                        emsg.data = strMsg
                        self.blobxydebug_msg_publisher.publish(emsg)
                        #self.get_logger().info(strMsg)
                    else: 
                        #self.get_logger().info(f"BLOB ERROR {blobHMin=} < {blobH=}  > {blobHMax=} {X_mFiltered=}")
                        pass
        else :
            self.get_logger().info("camera blob out of range")

    # called when the TOF sensor set is read
    # parse into a XY array to merge with openmv detect
    def tof8x8x3_msg_callback(self, msg) :
        x=-1 #skip 1st substring text
        y=0
        # parse message
        strArray = msg.data.split(" ")
        if strArray[0]!="TOF8x8x3" or len(strArray)!=193:
            self.get_logger().error(f"TOF8x8x3 message format error: {msg.data}")
            return
    
        try:
            for str in strArray :
                    if x+y>0: self.tofXY[y,x] = int(str)
                    if x<23 : x=x+1
                    else : 
                        x = 0
                        y = y+1

        except:
            self.get_logger().error(f"TOF8x8x3 message parse error: {msg.data}")
            return
        
        # create 8x24 "image" using text numbers and publish for debug
        strMsg = "\n"
        for y in range(0,8):
            for x in range(0,24):
                n = self.tofXY[y,x]
                if n<0 : strMsg = strMsg+"."
                elif n<100 : strMsg = strMsg+"0"
                elif n<200 : strMsg = strMsg+"1"
                elif n<500 : strMsg = strMsg+"2"
                elif n<1000 : strMsg = strMsg+"3"
                elif n<1500 : strMsg = strMsg+"4"
                elif n<2000 : strMsg = strMsg+"5"
                elif n<2500 : strMsg = strMsg+"6"
                else : strMsg = strMsg+"+"
            strMsg = strMsg+"\n"
        strMsg = strMsg+"|      |   ^^   |      |\n"

        msg = String()
        msg.data = strMsg
        self.tofxydebug_msg_publisher.publish(msg)

    # return (X, Y, theta) map location relative to openmv sensor
    # Uses can height for distance metric
    def mapXYFromBlobXH(self, blobX: int, blobH: int) :

        # calculate distance from camera using can blob height
        if blobH > 0 :
            dist = (1000.0 * 110.0/blobH) + 0 # 70 # mm
        else :
            dist = 0 # TODO: uses NaN?
 
        # calculate map Y using distance (X) and HFOV trig
        thetaY: float = self.HFOV * (float(blobX)/self.imgRngX)
        thetaY_rad: float = math.pi*thetaY/180

        #mapY = int((dist*(math.tan(thetaY_rad))) / math.sqrt(2)) # mm
        mapY: int = int(dist*(math.sin(thetaY_rad))) # mm
        mapX: int = int(dist*(math.cos(thetaY_rad))) # mm
        
        return (mapX,mapY,thetaY_rad)

    def broadcast_tf(self, parent, child, xyt ):
        now = self.get_clock().now()
        #now += rclpy.duration.Duration(nanoseconds = 100000000)
        #now = rclpy.time.Time()
        # Create and broadcast the transform message 
        tfs = TransformStamped()
        tfs.header.stamp = now.to_msg()
        tfs.header.frame_id = parent
        tfs._child_frame_id = child
        tfs.transform.translation.x = xyt[0]
        tfs.transform.translation.y = xyt[1]
        tfs.transform.translation.z = 0.0 #theta # for debug should be 0.0  

        q = quaternion_from_euler(0.0, 0.0, xyt[2]) #x,y,theta

        tfs.transform.rotation.x = q[0]
        tfs.transform.rotation.y = q[1]
        tfs.transform.rotation.z = q[2]
        tfs.transform.rotation.w = q[3]

#        self.get_logger().info(f"Broadcast {parent} {tfs = }".encode())

        self.tf_broadcaster.sendTransform(tfs)    


# simplified code for 2D robot
def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

# returns filtered data; delay is (length-1)/2 typ
def medianFilter(dataArray, data) :
    length = len(dataArray)
    # shift in new data, oldest data is discarded
    for i in range(1,length) :
        idx = length - i
        dataArray[idx] = dataArray[idx-1]
    dataArray[0] = data
    # sort data
    sortArray = dataArray.copy()
    sortArray.sort()
    filteredData = sortArray[int((length-1)/2)] # middle of sorted data (median)
    return (dataArray, filteredData)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = Robo24CanXYNode()
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

