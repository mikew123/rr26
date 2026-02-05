# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Single Color RGB565 Blob Tracking Example
#
# This example shows off single color RGB565 tracking using the OpenMV Cam.

import sensor
import time
import math

from machine import LED

led_red = LED("LED_RED")
led_green = LED("LED_GREEN")
led_blue = LED("LED_BLUE")


threshold_index = 4  # 0 for red, 1 for green, 2 for blue

# Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
# The below thresholds track in general red/green/blue things. You may wish to tune them...
# Range -128 to 127, (G,B,R)
thresholds = [
#    (30, 100,  15, 127, 15, 127),  #0 generic_red_thresholds
#    (30, 100, -64, -8, -32,  32),  #1 generic_green_thresholds
#    ( 0,  30,   0, 64,-128,   0),  #2 blue
#    (0, 100,  20,127,  0, 100),    #3 old 3M orange can??
    (0, 100,  15, 127, -128, 127), #4 new DUCK_orange_thresholds
    (0, 100,  25, 127, -128, 127), #5
    (0, 100,  30, 127, -128, 127), #6
    (0, 100,  35, 127, -128, 127), #7
    (0, 100,  40, 127, -128, 127), #8
    (0, 100,  45, 127, -128, 127), #9
]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.SVGA)
imgMaxX = 800 # SVGA horizontal
imgMaxY = 600 # SVGA vertical    limY = int(imgMaxY/1.5)
limY = int(imgMaxY/2)
verMid = (int(imgMaxX/2), 0, int(imgMaxX/2), imgMaxY)
horMid = (0 ,limY , imgMaxX, limY)

default_roi = (0, limY, imgMaxX, imgMaxY)

sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

blobDist = 10000
blobArea = 0

led_blue.on()

def blob_cb(blob) :
    global blobDist
    global blobArea
    retVal = True
    pi = 3.14159265

    #print(blob.code())

    # Test for vertical object
    m = blob.major_axis_line()
    # range = 0 to pi
    d = abs(math.atan2((m[3]-m[1]), (m[2]-m[0])))
    #convert range to 0 to pi/2
    if d > pi/2 : d = pi - d
    #test for close to 0 or pi/2 (90deg)
    piOff = pi/20
    if d < pi/4 : # 0 to pi/4
        if d > piOff :
            retVal = False
        retVal = False # Only vertical close to pi/2
    else : #pi/4 to pi/2
        if d < ((pi/2) - piOff) :  retVal = False

    if blob.compactness() < 0.2: retVal = False
    if blob.elongation()  > 0.9: retVal = False

    # Test for can shape 6.5X12
    #TODO: is elongnation test still needed?
    can_hw = 12.0/6.5
    can_obj_tol = 0.2 # tolerance of h/w test
    h = float(blob.h())
    w = float(blob.w())
    obj_hw = h/w
    can_obj = can_hw/obj_hw
    if math.fabs(1.0 - can_obj) > can_obj_tol :
        retVal = False

    x = -int(imgMaxX/2) + blob.cx()
    y = imgMaxY - blob.cy()
    d = math.sqrt((y*y) + (x*x)/10)
    if retVal == True :
        if d < blobDist :  blobDist = d

    a = blob.area()/100
    if retVal == True :
        if a > blobArea : blobArea = a

    #print(d,a,h,w,can_obj, retVal)

    return retVal
    #return False

################### MAIN LOOP #########################
roi = default_roi
obj_det_cnt = 0
f=0

while True:
    clock.tick()
    img = sensor.snapshot()
    n=0
    selectedObj=None
    for thr in thresholds :
        #print(f"{n=} {thr=}")
        blobDist = 10000
        blobArea = 0
        blobs = img.find_blobs(
            #[thresholds[5], thresholds[4]],
            [thr],
            # Filter out small objects
            pixels_threshold=1000,
            area_threshold=1000,
            merge=False,
            roi=roi,
            threshold_cb=blob_cb, # Other tests
        )


        for blob in blobs :
            obj = dict()

            x = -int(imgMaxX/2) + blob.cx()
            y = imgMaxY - blob.cy()
            d = math.sqrt((y*y) + (x*x)/10)
            a = blob.area()/100
            #if blobDist == d :
            if blobArea == a :
                # These values depend on the blob not being circular
                obj["x"] = -int(imgMaxX/2) + blob.cx()
                obj["y"] = imgMaxY - blob.cy()
                obj["a"] = blob.area()/100
                obj["n"] = n

                # calc distance and height of object
                m = blob.major_axis_line()
                obj["d"] = abs(math.atan2((m[3]-m[1]), (m[2]-m[0])))
                obj["h"] = int(math.sqrt(math.pow((m[0]-m[2]),2)+math.pow((m[1]-m[3]),2)))

                obj["maj"] = m
                obj["min"] = blob.minor_axis_line()
                obj["rec"] = blob.rect()

                #print(f"{obj=}")

                #if selectedObj==None or (obj["d"] < selectedObj["d"]) :
                if selectedObj==None :
                    selectedObj = obj
                elif obj["a"] > selectedObj["a"] :
                    selectedObj = obj
        n+=1

    #print(f"{selectedObj=}")

    # Draw image cross hairs after processing for blobs etc
    img.draw_line(verMid, color=(255,0,255), thickness=2)
    img.draw_line(horMid, color=(255,0,255), thickness=2)

    if selectedObj!=None :
        # create roi of 2X detected can size
        default_roi = (0, limY, imgMaxX, imgMaxY)
        can_rec = selectedObj["rec"]
        can_x = can_rec[0]
        can_y = can_rec[1]
        can_w = can_rec[2]
        can_h = can_rec[3]

        scale = 3
        roi_w = 300 #scale*can_w
        roi_h = 400 #scale*can_h
        roi_x = int((can_x+can_w/2)-(roi_w/2))
        roi_y = int((can_y+can_h/2)-(roi_h/2))
        roi = (roi_x,roi_y,roi_w,roi_h)

        # Draw can markers
        major = selectedObj["maj"]
        minor = selectedObj["min"]
        rectangle = selectedObj["rec"]
        img.draw_line(major, color=(0, 255, 0), thickness=4)
        img.draw_line(minor, color=(0, 0, 255), thickness=4)
        img.draw_rectangle(rectangle, thickness=2)
        img.draw_rectangle(roi, thickness=2)
        x = selectedObj["x"]
        y = selectedObj["y"]
        a = selectedObj["a"]
        h = selectedObj["h"]

        printStr = "SVGA %d %d %d %d %2.1f" % (x,y,a,h,f)
        print(printStr)
        #print("SVGA %d %d %d %d %2.1f" % (x,y,a,h,f))
        obj_det_cnt = 10

    else :
        if obj_det_cnt > 0 :
            obj_det_cnt-=1
            print(printStr)
            img.draw_line(major, color=(0, 255, 0), thickness=4)
            img.draw_line(minor, color=(0, 0, 255), thickness=4)

        #print(f"{obj_det_cnt=}")




    # reset roi to default if object not detected in a while
    if obj_det_cnt==0 :
        roi = default_roi

    f = clock.fps()


