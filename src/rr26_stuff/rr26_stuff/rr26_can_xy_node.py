#!/usr/bin/env python3

# 4/18/2025 MRW copied from robo24_can_xy_node.property

import rclpy
import math
import time
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from datetime import timedelta

class Robo24CanXYNode(Node):
    # parameters?
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

    def __init__(self):
        super().__init__('robo24_can_xy_node')

        self.openmv_msg_subscriber = self.create_subscription( String, 'openmv_msg', self.openmv_msg_callback, 10)
        self.tof8x8x3_msg_subscriber = self.create_subscription( String, 'tof8x8x3_msg', self.tof8x8x3_msg_callback, 10)
        self.tofxydebug_msg_publisher = self.create_publisher(String, 'tofxydebug_msg', 10)
        self.blobxydebug_msg_publisher = self.create_publisher(String, 'blobxydebug_msg', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Robo24CanXYNode Started")

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

    node = Robo24CanXYNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

# This code is needed to run .py file directly
if __name__ == '__main__':
    main()

