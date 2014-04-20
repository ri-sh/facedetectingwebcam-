#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a camera image or video stream and displays a red box around them,
then centres the webcam via two servos so the face is at the centre of the screen
Based on facedetect.py in the OpenCV samples directory
"""
import sys
from optparse import OptionParser
import time
import math
import datetime
import serial
import cv
 
# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned
# for accurate yet slow object detection. For a faster operation on real video
# images the settings are:
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING,
# min_size=<minimum possible face size
 
min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0
 
def detect_and_draw(img, cascade):
    gray = cv.CreateImage((img.width,img.height), 8, 1)
    small_img = cv.CreateImage((cv.Round(img.width / image_scale),
                   cv.Round (img.height / image_scale)), 8, 1)
 
    # convert color input image to grayscale
    cv.CvtColor(img, gray, cv.CV_BGR2GRAY)
 
    # scale input image for faster processing
    cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)
 
    cv.EqualizeHist(small_img, small_img)
 
    centre = None
 
    if(cascade):
        t = cv.GetTickCount()
        # HaarDetectObjects takes 0.02s
        faces = cv.HaarDetectObjects(small_img, cascade, cv.CreateMemStorage(0),
                                     haar_scale, min_neighbors, haar_flags, min_size)
        t = cv.GetTickCount() - t
        if faces:
            for ((x, y, w, h), n) in faces:
                # the input to cv.HaarDetectObjects was resized, so scale the
                # bounding box of each face and convert it to two CvPoints
                pt1 = (int(x * image_scale), int(y * image_scale))
                pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
                cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)
                # get the xy corner co-ords, calc the centre location
                x1 = pt1[0]
                x2 = pt2[0]
                y1 = pt1[1]
                y2 = pt2[1]
                centrex = x1+((x2-x1)/2)
                centrey = y1+((y2-y1)/2)
                centre = (centrex, centrey)
 
    cv.ShowImage("result", img)
    return centre
 
def move_servos(xygo):
    position = '%da%db' % (xygo[0], xygo[1])
    ser.write(position)
 
def get_delta(loc, span, max_delta, centre_tolerance):
    """How far do we move on this axis to get the webcam
       centred on the face?
       loc is the face's centre for this axis
       span is the width or height for this axis
       max_delta is the max nbr of degrees to move on this axis
       centre_tolerance is the centre region where we don't allow movement
       """
    framecentre = span/2
    delta = framecentre - loc
    if abs(delta) < centre_tolerance: # within X pixels of the centre
        delta = 0 # so don't move - else we get weird oscillations
    else:
        is_neg = delta <= 0
        to_get_near_centre = abs(delta) - centre_tolerance
        if to_get_near_centre > 35:
            delta = 4
        else:
            # move slower if we're closer to centre
            if to_get_near_centre > 25:
                delta = 3
            else:
                # move real slow if we're very near centre
                delta = 1
        if is_neg:
            delta = delta * -1
    return delta
 
if __name__ == '__main__':
    # open a serial port
    ser=serial.Serial(port='/dev/cu.usbserial',baudrate=19200,timeout=0)
    ser.write('r')
    xygo = (90,90)
    move_servos(xygo)
 
    # parse cmd line options, setup Haar classifier
    parser = OptionParser(usage = "usage: %prog [options] [camera_index]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = "/Users/ian/Documents/OpenCV-2.1.0//data/haarcascades/haarcascade_frontalface_alt.xml")
    (options, args) = parser.parse_args()
 
    cascade = cv.Load(options.cascade)
 
    if len(args) != 1:
        parser.print_help()
        sys.exit(1)
 
    input_name = args[0]
    if input_name.isdigit():
        capture = cv.CreateCameraCapture(int(input_name))
    else:
        print "We need a camera input! Specify camera index e.g. 0"
        sys.exit(0)
 
    cv.NamedWindow("result", 1)
 
    if capture:
        frame_copy = None
 
        while True:
            frame = cv.QueryFrame(capture)
            if not frame:
                cv.WaitKey(0)
                break
            if not frame_copy:
                frame_copy = cv.CreateImage((frame.width,frame.height),
                                            cv.IPL_DEPTH_8U, frame.nChannels)
            if frame.origin == cv.IPL_ORIGIN_TL:
                cv.Copy(frame, frame_copy)
            else:
                cv.Flip(frame, frame_copy, 0)
 
            centre = detect_and_draw(frame_copy, cascade)
 
            if centre is not None:
                cx = centre[0]
                cy = centre[1]
 
                # modify the *-1 if your x or y directions are reversed!
                xdelta = get_delta(cx, frame_copy.width, 6, 15) * -1
                ydelta = get_delta(cy, frame_copy.height, 1, 25) * -1
 
                # on my camera I introduce a delay after movements
                # else my assembly wobbles and the webcam transmits
                # a non-centred image, so weird oscillations can occur
                total_delta = abs(xdelta)+abs(ydelta)
                if total_delta > 0:
                    xygo = (xygo[0]+xdelta,xygo[1]+ydelta)
 
                    sleep_for = 1/10.0*min(total_delta, 10)
                    sleep_for = min(sleep_for, 0.4)
 
                    move_servos(xygo)
                else:
                    sleep_for = 0
 
            if cv.WaitKey(10) >= 0: # 10ms delay
                break
 
    cv.DestroyWindow("result")
