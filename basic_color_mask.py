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
lowH = 0
highH = 179
lowS = 0
highS = 255
lowV = 0
highV = 255
# objects
webcam = None
pub = None
# webcam info
cam_width = 1
cam_height = 1

def nothing(x):
    pass

def updateHSV():
    """Reads the image color tuning trackbar and updates global data"""
    global lowH
    global highH
    global lowS
    global highS
    global lowV
    global highV
    
    lowH  = cv2.getTrackbarPos('lowH','control')
    highH = cv2.getTrackbarPos('highH','control')
    lowS  = cv2.getTrackbarPos('lowS','control')
    highS = cv2.getTrackbarPos('highS','control')
    lowV  = cv2.getTrackbarPos('lowV','control')
    highV = cv2.getTrackbarPos('highV','control')

def update_motors(x, y):
    """
    Given the x and y coordinates of the color blob,
    update the left and right motor speeds.
    """
    # calculate blob's displacement from horizontal center of image (1-dimensional)

    # if the coordinates are negative (meaning invalid), set a medium-speed CCW pivot (no forward motion)

    # if displacement is within accepted range, run motors at the same speed
    # if displacement is negative, turn left by slowing the left tread and speeding up the right tread
    # else (if displacement is positive) turn right by slowing the right tread and speeding up the left tread 

    # scale the speed difference by the magnitude of the displacement?
    pass

def run():
    """Main image masking and publishing code"""
    while True:
        # read frame from webcam
        _,img = webcam.read()
        # update color tuning from control panel
        updateHSV()
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

        binary = cv2.dilate(binary,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))
        binary = cv2.erode(binary,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)))

        center_x = -1
        center_y = -1
        # get moments of image
        moments = cv2.moments(binary)
        if (moments['m00'] > 0.0):
            # find the "center of gravity" of the moment 
            # (which is hopefully the tracked object)
            center_x = int(moments['m10']/moments['m00'])
            center_y = int(moments['m01']/moments['m00'])

        # draw a green dot on the center and display images on local screen
        cv2.circle(mask,(center_x,center_y),2,[0,255,0],2)
        cv2.imshow('masked',mask)
        cv2.imshow('binary',binary)
	cv2.waitKey(1)

def setup_control_panel():
    """Setup color tuning panel"""
    cv2.namedWindow('control',flags=cv2.WINDOW_NORMAL)
    cv2.createTrackbar('lowH','control',0,179,nothing)
    cv2.createTrackbar('highH','control',179,179,nothing)
    cv2.createTrackbar('lowS','control',0,255,nothing)
    cv2.createTrackbar('highS','control',255,255,nothing)
    cv2.createTrackbar('lowV','control',0,255,nothing)
    cv2.createTrackbar('highV','control',255,255,nothing)

def set_img_dimensions():
    """Set dimensions of frame captured from webcam"""
    global cam_width
    global cam_height
    _,img = webcam.read()
    cam_height,cam_width,_ = img.shape


def init():
    """Initialize and run the program"""
    global webcam
    cv2.namedWindow('masked')
    cv2.namedWindow('binary')
    setup_control_panel()
    webcam = cv2.VideoCapture(0)
    set_img_dimensions()
    run()


if __name__ == '__main__':
    init()

