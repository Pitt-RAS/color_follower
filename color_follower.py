#!/usr/bin/env python 
"""
This color following code is based on the work of github user ClintLiddick.

The script uses OpenCV2 to capture webcam input, convert it to HSV, 
threshold and mask the images, and find the center of a blob of the
color.

The displacement of the blob from the center coordinates is used to decide
motor direction and speed, which is then set using the motor outputs and
PWM pins.
"""

import cv2
import numpy
import Adafruit_BBIO.GPIO as GPIO
import Adafruit_BBIO.PWM as PWM

"""
Output pins for the motor driver board:
"""
motor_pwms = ["P9_22", "P9_21"]   #[M1D2, M2D2], 
motor_ins = [["P9_23", "P9_24"], ["P9_25", "P9_26"]]    # [[M1IN1, M1IN2], [M2IN1, M2IN2]]
STBY = "P9_27"  # Connect to "EN" pin on Pololu Dual MC33926 board


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
TURN_SPEED = 25.0
FORWARD = 1
BACKWARD = -1
LEFT = 0
RIGHT = 1

samples_without_find = 0

def nothing(x):
    pass


def init_motors():
    """
    Initialize the pins needed for the motor driver.
    """
    global motor_ins
    global motor_pwms
    # initialize GPIO pins
    GPIO.setup(STBY, GPIO.OUT)
    GPIO.output(STBY, GPIO.HIGH)
    for motor in motor_ins:
        for pin in motor:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

    # initialize PWM pins
    # first need bogus start due to unknown bug in library
    PWM.start("P9_14", 50.0)
    PWM.stop("P9_14")
    # now start the desired PWMs
    for pwm_pin in motor_pwms:
        PWM.start(pwm_pin, 0.0)
        PWM.set_run(pwm_pin, 1)

def set_motor(motor, direction, value):
    """
    Set an individual motor's direction and speed
    """
    if direction == BACKWARD: # For now, assume CW is forwards
        # forwards: in1 LOW, in2 HIGH
        GPIO.output(motor_ins[motor][0], GPIO.LOW)
        GPIO.output(motor_ins[motor][1], GPIO.HIGH)
    elif direction == FORWARD:
        GPIO.output(motor_ins[motor][0], GPIO.HIGH)
        GPIO.output(motor_ins[motor][1], GPIO.LOW)
    else:
        # there has been an error, stop motors
        GPIO.output(STBY, GPIO.LOW)
    PWM.set_duty_cycle(motor_pwms[motor], value)



def update_motors(x, y):
    """
    Given the x and y coordinates of the color blob,
    update the left and right motor speeds.
    """
    # calculate blob's displacement from horizontal center of image (1-dimensional)
    displacement = x - cam_cx
   
    # if displacement is within accepted range, run motors at the same speed
    if abs(displacement) <= DEAD_ZONE:
        print 'move forward'
        set_motor(LEFT, FORWARD, FORWARD_SPEED)
        set_motor(RIGHT, FORWARD, FORWARD_SPEED)
    elif displacement < 0:
        print 'turn left'
        set_motor(LEFT, BACKWARD, TURN_SPEED)
        set_motor(RIGHT, FORWARD, TURN_SPEED)
    else:
        set_motor(LEFT, FORWARD, TURN_SPEED)
        set_motor(RIGHT, BACKWARD, TURN_SPEED)
        print 'turn right'

    # TODO: scale the speed difference by the magnitude of the displacement?
 

def run():
    """Main image masking and publishing code"""
    global samples_without_find
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

def close_all():
    """
    Clean up all motor outputs
    """
    GPIO.output(STBY, GPIO.LOW)
    for motor in motor_ins:
        for pin in motor:
            GPIO.output(pin, GPIO.LOW)
    for pwm_pin in motor_pwms:
        PWM.stop(pwm_pin)
    

def init():
    """Initialize and run the program"""
    global webcam
    #setup_control_panel()
    webcam = cv2.VideoCapture(0)
    webcam.set(3, 320)
    webcam.set(4, 240)
    set_img_dimensions()
    init_motors()
    try:
        run()
    except:
        close_all()
    exit()

if __name__ == '__main__':
    init()

