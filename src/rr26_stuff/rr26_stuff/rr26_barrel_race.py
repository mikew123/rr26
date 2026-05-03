import math
import time

import tf_transformations

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped

from robo24_interfaces.msg import Barrels, Barrel

class BarrelRace() :
    """
    Barrel race class used by rr26_controller_node

    """

    def __init__(self, ctrl_self) :
        self.ctrl = ctrl_self

        # get links to functions in ROS2 rr26_controller_node
        self.get_logger = self.ctrl.get_logger
        self.getAngleDist2CanBlob = self.ctrl.getAngleDist2CanBlob
        # self.lidarDistToBarrel = self.ctrl.lidarDistToBarrel
        self.cmd_vel_publisher = self.ctrl.cmd_vel_publisher
        self.get_clock = self.ctrl.get_clock
        self.getCurrentPose = self.ctrl.getCurrentPose
        
        self.get_logger().info(f"BarrelRace class module init")

    # Barrel racing global variables
    enable_br_states = False
    brTimer = 0.0
    curr_brState:str = ""
    next_brState:str = "init"
    brCnt = 0
    currentAngVel = 0.0
    currentLinVel = 0.0
    
    def runBarrelRace(self) :
        """
        The barrel race is started by calling this function
        This function is blocking and returns when the barrel race is finished
        """
        
        self.get_logger().info(f"barrel_race.runBarrelRace: started (button Y)")

        # Enables the Barrel Race can statemachine running in scan (Lidar) callback
        self.enable_br_states = True

        # Barrel Race runs when enable state is True using the scan (Lidar) callback
        while self.enable_br_states==True :
            time.sleep(0.1)
                 
        self.get_logger().info(f"barrel_race.runBarrelRace: Barrel Race state machine finished")


    def ft2m(self, ft: float) -> float:
        return ft * 0.3048

    def lidarDistToBarrel(self, barrels:Barrels, dmax, amin, amax) -> Barrel :
        """
        get distance and angle to the barrel using the lidar can detection
        relative to the robot center
        Select the barrel that meets the distance and angle requirements
        return angle, dist to barrel relative to center of robot
        """

        # convert angles from degrees to radians
        amin = math.radians(float(amin))
        amax = math.radians(float(amax))

        barrelDetected:Barrel =  None
        

        if  len(barrels.barrel) >0 :
            for barrel in barrels.barrel :
                a = barrel.angle
                d = barrel.distance
                if (d<=dmax) and (a>amin) and (a<amax) :
                    barrelDetected = barrel
                    break
        
        # self.get_logger().info(f"lidarDistToBarrel: {barrels=} {barrelDetected=}")

        return barrelDetected
    
    def barrels_callback(self, msg:Barrels) -> None:
        """
        Barrel racing is run in the /barrels topic callback which is published
        every /scan Lidar topic (~10Hz).
        The /barrels topic has a list of detected barrels distance and angle 
        from the robot center (Lidar)

        """

        # rename Barrels msg
        barrels = msg

        # estimated sensor lag
        odomLag:float  = 0.1
        camLag:float   = 0.2
        lidarLag:float = 0.1

        # get the current velocities from the last velocity command
        currentAngVel = self.currentAngVel
        currentLinVel = self.currentLinVel

        # default no movement
        linX:float = 0.0
        angZ:float = 0.0

        # nominal linear and angular velocity when going around barrels
        linX0: float    = 0.30
        angZ0: float    = 3*linX0
        angScale: float = 10*linX0

        # set distance and angle from barrel when going around it
        distFromBarrel = 0.25 # center of robot to center of can
        angleToBarrel = math.radians(90.0)

        # # initialize when the button is pressed
        # if self.gotoBarrelRace==True and self.gotoBarrelRace_last==False :
        #     self.barrelRaceActive = True

        a = 0.0
        d = 0.0
        aa = 0.0 
        dd = 0.0       
        
        stateChange = False

        if self.enable_br_states==True :
            current_time = self.get_clock().now()

            if self.curr_brState!=self.next_brState :
                stateChange = True
                self.brStartTime = current_time
                self.brCounter = 0

                self.get_logger().info(f"barrel_race.barrels_callback: state change {self.curr_brState} -> {self.next_brState}")

            elapsed_time = (current_time - self.brStartTime).nanoseconds / 1e9

            state:str = self.next_brState
            next_state:str = state # default no state change

            # get current robot pose x,y,angle from start
            current_pose:PoseStamped=None
            q:Quaternion=None
            (tf_OK,current_pose) = self.getCurrentPose()
            #TODO: test for tf_OK
            q = current_pose.pose.orientation
            # convert quaterion to euler
            (_,_,z) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            currentAngle = z
            currentX = current_pose.pose.position.x
            currentY = current_pose.pose.position.y

            # correct odom angle for rotation velocity
            currentAngle += currentAngVel * odomLag

            # correct odom XY location with linear velocity
            # TODO: use corrected Angle or non-corrected 
            velX = currentLinVel * math.cos(currentAngle)
            velY = currentLinVel * math.sin(currentAngle)
            currentX += velX * odomLag
            currentY += velY * odomLag

        # STATE init
            if state=="init" :
                # Initialize 
                next_state = "start"
            
        # STATE start
            elif state=="start" :
                # Drive past start line
                linX = linX0
                angZ = 0.0
                dist = self.ft2m(2.5)
                timeout = dist/linX

                if elapsed_time>=timeout:
                    next_state = "gotoB1A"

        # STATE gotoB1A
            elif state=="gotoB1A" :
                # Drive in arc towards Can 1, stop when pointed towards barrel1
                # the camera should be able to see the can in the ranges of placement
                linX = linX0
                targetAngle = math.radians(65.0)
                dArc = 0.75 # arc radius in ft
                # set timeout
                dMax = 2.0 # travel meters before timeout
                tMax = dMax/linX

                if elapsed_time>=tMax :
                    linX = 0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif currentAngle>=targetAngle :
                    self.get_logger().info(f"barrel_race.barrels_callback: pointed toward barrel 1 {state=} {elapsed_time=} {currentAngle=}")
                    next_state = "gotoB1B"

                else :
                    angZ = (linX * targetAngle)/self.ft2m(dArc)

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {targetAngle=} {linX=} {angZ=}")
                
        # STATE gotoB1B
            elif state=="gotoB1B" :
                # Drive to get close to barrel1 using camera blob detection
                linX = linX0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                (tf_OK, a, d) = self.getAngleDist2CanBlob()
                # attempt to correct can detect angle 
                a -= currentAngVel * camLag
                # offset detect angle to point toward side of can
                a += math.radians(-20.0)

                # Barrel detection using Lidar
                barrel = self.lidarDistToBarrel(barrels, dmax=1.0, amin=-50, amax=50)

                if elapsed_time>=tMax :
                    linX=0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif barrel!=None :
                    aa = barrel.angle
                    dd = barrel.distance
                    if dd < 0.75 :
                        self.get_logger().info(f"barrel_race.barrels_callback: Lidar detected barrel 1 is close {state=} {elapsed_time=} {aa=} {dd=}")
                        next_state = "aroundB1A"

                elif tf_OK == False :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 1 not detected with cam blob, ignore {state=} {elapsed_time=}")
                    
                elif d > 2.0 :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 1 dist too far, ignore {state=} {elapsed_time=}")

                elif d> 0.75 :
                    aDiff = a - math.radians(0.0)
                    # Head toward barrel
                    angZ = aDiff *(1) #TODO scale with linX

                else :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 1 is close {state=} {elapsed_time=} {a=} {d=}")
                    next_state = "aroundB1A"

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {tf_OK=} {a=} {d=} {aa=} {dd=} {linX=} {angZ=}")

        # STATE aroundB1A
            elif state=="aroundB1A" :
                # Drive around barrel 1 using lidar to get next to barrel
                # nominal speeds to circle barrel CCW
                linX = linX0
                angZ = angZ0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                # Qualify barrels loosely
                barrel = self.lidarDistToBarrel(barrels, dmax=1.5, amin=-90, amax=135)
                if barrel!=None:
                    a = barrel.angle
                    d = barrel.distance
                    # attempt to correct lidar barrel detect angle
                    a -= currentAngVel * lidarLag

                if elapsed_time >= tMax :
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif barrel==None :
                    # ignore no barrel detect
                    self.get_logger().info(f"barrel_race.barrels_callback: no barrel detected {state=} {elapsed_time=}")

                elif a<math.radians(100.0) and a>math.radians(80.0) and d<0.5:
                    self.get_logger().info(f"barrel_race.barrels_callback: got next to barrel 1 {state=} {elapsed_time=} {currentAngle=}")
                    next_state = "aroundB1B"

                else :
                    angZ += angScale*(d - distFromBarrel)
                    angZ += angScale*(a - angleToBarrel)

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {barrel=} {linX=} {angZ=}")

        # STATE aroundB1B
            elif state=="aroundB1B" :
                # Drive around barrel 1 using lidar
                # nominal speeds to circle barrel CCW
                linX = linX0
                angZ = angZ0
                angExit = -90
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                a = 0.0
                d = 0.0

                # Qualify barrel detection tightly while driving around the barrel
                barrel = self.lidarDistToBarrel(barrels, dmax=0.6, amin=45, amax=135)
                if barrel!=None :
                    a = barrel.angle
                    d = barrel.distance
                    # attempt to correct lidar barrel detect angle
                    a -= currentAngVel * lidarLag

                if elapsed_time >= tMax :
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif barrel==None :
                    # no barrel detected - coast at nominal speeds
                    self.get_logger().info(f"barrel_race.barrels_callback: no barrel detected {state=} {elapsed_time=}")

                elif currentAngle<0 and currentAngle>math.radians(angExit) :
                    # stop angular rotation - continue straight
                    linX = 0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: went around barrel 1 {state=} {elapsed_time=} {currentAngle=}")
                    next_state = "gotoB2A"

                else :
                    # control angular ratation to drive around the barrel
                    angZ += angScale*(d - distFromBarrel)
                    angZ += angScale*(a - angleToBarrel)

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {barrel=} {linX=} {angZ=}")

        # STATE gotoB2A
            elif state=="gotoB2A" :
                # Drive toward barrel 2 in straight line to get closer, stop after distance
                linX = linX0

                targetDist = 0.75
                driveTime = targetDist/linX0

                if elapsed_time<driveTime :
                    # Head toward barrel
                    angZ = 0.0
                else :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 2 is close {state=} {elapsed_time=}")
                    next_state = "gotoB2B"

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {driveTime=} {linX=} {angZ=}")

        # STATE gotoB2B
            elif state=="gotoB2B" :
                # Drive toward barrel 2 using camera blob detection, stop at distance
                linX = linX0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                (tf_OK, a, d) = self.getAngleDist2CanBlob()
                # attempt to correct can detect angle 
                a -= currentAngVel * camLag
                # offset detect angle to point toward side of can
                a += math.radians(20.0)

                # Barrel detection using Lidar
                barrel = self.lidarDistToBarrel(barrels, dmax=1.0, amin=-50, amax=50)

                if elapsed_time>=tMax :
                    linX = 0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif barrel!=None :
                    aa = barrel.angle
                    dd = barrel.distance
                    if dd < 0.75 :
                        self.get_logger().info(f"barrel_race.barrels_callback: Lidar detected barrel 1 is close {state=} {elapsed_time=} {aa=} {dd=}")
                        next_state = "aroundB2A"

                elif d > 2.0 :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 2 dist too far, ignore {state=} {elapsed_time=}")

                elif tf_OK == False :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel not detected with cam blob, ignore {state=} {elapsed_time=}")
                    
                elif d > 0.75 :
                    # Head toward barrel
                    angZ = angScale*a

                else :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 2 is close {state=} {elapsed_time=} {a=} {d=}")
                    next_state = "aroundB2A"

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {linX=} {angZ=}")

        # STATE aroundB2A
            elif state=="aroundB2A" :
                # Drive around barrel 1 using lidar to go to left side of barrel
                # nominal speeds to circle barrel CW
                linX = linX0
                angZ = -angZ0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                barrel = self.lidarDistToBarrel(barrels, dmax=1.5, amin=-135, amax=90)
                if barrel!=None:
                    a = barrel.angle
                    d = barrel.distance
                    # attempt to correct lidar barrel detect angle
                    a -= currentAngVel * lidarLag

                if elapsed_time >= tMax :
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    linX = 0.0
                    angZ = 0.0
                    next_state = "end"

                elif barrel==None :
                    # no barrel detected - coast at nominal speeds
                    self.get_logger().info(f"barrel_race.barrels_callback: no barrel detected {state=} {elapsed_time=}")

                elif a>math.radians(-100.0) and a<math.radians(-80.0) and d<0.5:
                    # coast at nominal speeds
                    self.get_logger().info(f"barrel_race.barrels_callback: drove the side of barrel 2 {state=} {elapsed_time=} {currentAngle=}")
                    next_state = "aroundB2B"
                else :
                    # control angular rotation to drive around the barrel
                    angZ -= angScale*(d - distFromBarrel)
                    angZ += angScale*(a + angleToBarrel)

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {barrel=} {linX=} {angZ=}")

        # STATE aroundB2B
            elif state=="aroundB2B" :
                # Drive around barrel 1 using lidar to go arround the barrel
                # nominal speeds to circle barrel CW
                linX = linX0
                angZ = -angZ0
                angExit = 30.0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                # tighter barrel detect assume close to the side
                barrel = self.lidarDistToBarrel(barrels, dmax=0.6, amin=-135, amax=-45)
                if barrel!=None :
                    a = barrel.angle
                    d = barrel.distance
                    # attempt to correct lidar barrel detect angle
                    a -= currentAngVel * lidarLag

                if elapsed_time >= tMax :
                    linX = 0.0
                    angZ = 0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif barrel==None :
                    # no barrel detected - coast around barrel
                    self.get_logger().info(f"barrel_race.barrels_callback: no barrel detected {state=} {elapsed_time=}")

                elif currentAngle>0 and currentAngle<math.radians(angExit) :
                    # stop angular rotation - continue straight
                    angZ = 0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: went around barrel 2 {state=} {elapsed_time=} {currentAngle=}")
                    next_state = "gotoB3A"

                else :
                    # control angular rotation to drive around the barrel
                    angZ -= angScale*(d - distFromBarrel)
                    angZ += angScale*(a + angleToBarrel)

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {barrel=} {linX=} {angZ=}")

        # STATE gotoB3A
            elif state=="gotoB3A" :
                # Drive toward barrel 3 in straight line to get closer, stop after distance
                linX = linX0

                targetDist = 0.75
                driveTime = targetDist/linX

                if elapsed_time<driveTime :
                    # Head toward barrel
                    angZ = 0.0

                else :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 3 is close {state=} {elapsed_time=}")
                    next_state = "gotoB3B"

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {driveTime=} {linX=} {angZ=}")

        # STATE gotoB3B
            elif state=="gotoB3B" :
                # Drive toward barrel 3 using camera blob detection, stop at distance
                linX = linX0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                (tf_OK, a, d) = self.getAngleDist2CanBlob()
                # attempt to correct can detect angle 
                a -= currentAngVel * camLag
                # offset detect angle to point toward side of can
                a += math.radians(20.0)

                # Barrel detection using Lidar
                barrel = self.lidarDistToBarrel(barrels, dmax=1.0, amin=-50, amax=50)

                if elapsed_time>=tMax :
                    linX = 0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif barrel!=None :
                    aa = barrel.angle
                    dd = barrel.distance
                    if dd < 0.75 :
                        self.get_logger().info(f"barrel_race.barrels_callback: Lidar detected barrel 1 is close {state=} {elapsed_time=} {aa=} {dd=}")
                        next_state = "aroundB3A"

                elif d > 2.0 :
                    # coast
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 2 dist too far, ignore {state=} {elapsed_time=}")

                elif tf_OK == False :
                    # coast
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel not detected with cam blob, ignore {state=} {elapsed_time=}")
                    
                elif d > 0.75 :
                    # Head toward barrel
                    angZ = angScale * a

                else :
                    self.get_logger().info(f"barrel_race.barrels_callback: barrel 2 is close {state=} {elapsed_time=} {a=} {d=}")
                    next_state = "aroundB3A"

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {linX=} {angZ=}")

        # STATE aroundB3A
            elif state=="aroundB3A" :
                # Drive around barrel 3 using lidar to go to left side of barrel
                # nominal speeds to circle barrel CW
                linX = linX0
                angZ = -angZ0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                barrel = self.lidarDistToBarrel(barrels, dmax=1.5, amin=-135, amax=90)
                if barrel!=None:
                    a = barrel.angle
                    d = barrel.distance
                    # attempt to correct lidar barrel detect angle
                    a -= currentAngVel * lidarLag

                if elapsed_time >= tMax :
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    next_state = "end"

                elif barrel==None :
                    # no barrel detected - coast around barrel
                    self.get_logger().info(f"barrel_race.barrels_callback: no barrel detected {state=} {elapsed_time=}")

                elif a>math.radians(-100.0) and a<math.radians(-80.0) and d<0.5:
                    # continue coast around barrel
                    self.get_logger().info(f"barrel_race.barrels_callback: drove the side of barrel 3 {state=} {elapsed_time=} {currentAngle=}")
                    next_state = "aroundB3B"

                else :
                    # control angular rotation to drive around the barrel
                    angZ -= angScale*(d - distFromBarrel)
                    angZ += angScale*(a + angleToBarrel)

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {barrel=} {linX=} {angZ=}")

        # STATE aroundB3B
            elif state=="aroundB3B" :
                # Drive around barrel 3 using lidar to go around the barrel CW
                linX = linX0
                angZ = -angZ0
                # exit angle should be 180 but there is a discontinuity at 180
                angExit = 175 # needs more overshoot space for higher speeds
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                # tighter barrel detect assume close to the side
                barrel = self.lidarDistToBarrel(barrels, dmax=0.6, amin=-135, amax=-45)
                if barrel!=None :
                    a = barrel.angle
                    d = barrel.distance
                    # attempt to correct lidar barrel detect angle
                    a -= currentAngVel * lidarLag

                if elapsed_time >= tMax :
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    linX = 0.0
                    angZ = 0.0
                    next_state = "end"

                elif barrel==None :
                    # no barrel detected - coast around barrel
                    self.get_logger().info(f"barrel_race.barrels_callback: no barrel detected {state=} {elapsed_time=}")

                elif currentAngle>math.radians(90) and currentAngle<=math.radians(angExit) :
                    # stop angular rotation when pointing to start line - continue straight
                    angZ = 0.0
                    self.get_logger().info(f"barrel_race.barrels_callback: went around barrel 3 {state=} {elapsed_time=} {currentAngle=}")
                    next_state = "gotoMid"

                else :
                    # control angular rotation to drive around the barrel
                    angZ -= angScale*(d - distFromBarrel)
                    angZ += angScale*(a + angleToBarrel)

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {barrel=} {linX=} {angZ=}")

        # STATE gotoMid
            elif state=="gotoMid" :
                # head towards the middle between barrels 1 and 2 (maybe map x=4ft, y=0)
                linX = linX0
                dist = 1.0
                t = dist/linX

                if elapsed_time > t :
                    next_state = "gotoHome"

        # STATE gotoHome
            elif state=="gotoHome" :
                # Go to the begin point map x=0, y=0, heading 180 deg
                linX = linX0
                # set timeout
                dMax = 2.0
                tMax = dMax/linX

                if elapsed_time >= tMax :
                    self.get_logger().info(f"barrel_race.barrels_callback: timeout {state=} {elapsed_time=}")
                    linX = 0.0
                    angZ = 0.0
                    next_state = "end"

                elif currentAngle > 0 :
                    aDiff = (math.radians(180) - currentAngle)
                    
                else :
                    aDiff = (math.radians(-180) - currentAngle)

                if currentX > 0.0 :
                    # vear toward center line and angle due "south"
                    angZ += linX0 * aDiff   
                    angZ += 10*linX0 * currentY

                else :
                    linX = 0.0
                    angZ = 0.0
                    next_state = "spin"

                self.get_logger().info(f"barrel_race.barrels_callback: {state=} {elapsed_time=} {currentAngle=} {currentX=} {currentY=} {aDiff=} {linX=} {angZ=}")


        # STATE spin
            elif state=="spin" :
                # spin 180 to point towards barrel field
                spinAngle = 3.14
                spinTime = 5.0

                linX = 0.0
                angZ = spinAngle/spinTime

                if elapsed_time > spinTime :
                    next_state = "end"

        # STATE end
            elif state=="end" :
                linX = 0.0
                angZ = 0.0
                next_state = "init"
                self.enable_br_states = False


            # update robot movement
            cmd_vel = Twist()
            cmd_vel.linear.x = linX
            cmd_vel.angular.z = angZ
            self.cmd_vel_publisher.publish(cmd_vel)

            # Update persistant state variables
            self.next_brState = next_state
            self.curr_brState = state
            
            self.currentAngVel = angZ
            self.currentLinVel = linX
