#!/usr/bin/env python 
"""
This color following code is based on the work of github user ClintLiddick.

The script uses OpenCV2 to capture webcam input, convert it to HSV, 
threshold and mask the images, and find the center of a blob of the
color (which is set by tuning the trackbars).
"""

import cv2
import numpy

# OpenCV HSV value ranges
# right now follows yellow - needs to be hand calibrated before each run due to lighting and camera changes
# TODO: command line arguments?
lowH = 8
highH = 31
lowS = 108
highS = 243
lowV = 45
highV = 203
# objects
webcam = None
pub = None
# webcam info
cam_width = 1
cam_height = 1

cam_cx = 160
cam_cy = 120

DEAD_ZONE = 10
FORWARD_SPEED = 50.0

samples_without_find = 0

def nothing(x):
    pass

def update_motors(x, y):
    """
    Given the x and y coordinates of the color blob,
    update the left and right motor speeds.
    """
    # calculate blob's displacement from horizontal center of image (1-dimensional)
    displacement = x - cam_cx
    # if the coordinates are negative (meaning invalid), set a medium-speed CCW pivot (no forward motion)
   
    # if displacement is within accepted range, run motors at the same speed
    if abs(displacement) <= DEAD_ZONE:
        print 'move forward'
    # if displacement is negative, turn left by slowing the left tread and speeding up the right tread
    elif displacement < 0:
        print 'turn left'
    # else (if displacement is positive) turn right by slowing the right tread and speeding up the left tread 
    else:
        print 'turn right'
    # TODO: scale the speed difference by the magnitude of the displacement?
 

def run():
    global samples_without_find
    """Main image masking and publishing code"""
    while True:
        # read frame from webcam
        _,img = webcam.read()
        # convert frame to HSV format
        hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        # create mask for color selected in color tuning panel
        mask = cv2.inRange(hsv_img, numpy.array([lowH,lowS,lowV],numpy.uint8),\
                numpy.array([highH,highS,highV],numpy.uint8))
        # convert mask to binary image format
        _,binary = cv2.threshold(mask,127,255,cv2.THRESH_BINARY)
        # filter image to reduce noise
        binary = cv2.erode(binary,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
        binary = cv2.dilate(binary,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))

        center_x = -1
        center_y = -1
        # get moments of image
        moments = cv2.moments(binary)
        if (moments['m00'] > 0.0):
            samples_without_find = 0
            # find the "center of gravity" of the moment 
            # (which is hopefully the tracked object)
            center_x = int(moments['m10']/moments['m00'])
            center_y = int(moments['m01']/moments['m00'])
            update_motors(center_x, center_y)
        else:
            samples_without_find += 1
            print "Color not found."
            if samples_without_find > 50:
                # pivot in place (send bogus coords to robot)
                update_motors(-1, -1)


def set_img_dimensions():
    """Set dimensions of frame captured from webcam"""
    global cam_width
    global cam_height
    _,img = webcam.read()
    cam_height,cam_width,_ = img.shape


def init():
    """Initialize and run the program"""
    global webcam
    #setup_control_panel()
    webcam = cv2.VideoCapture(0)
    webcam.set(3, 320)
    webcam.set(4, 240)
    set_img_dimensions()
    run()


if __name__ == '__main__':
    init()

