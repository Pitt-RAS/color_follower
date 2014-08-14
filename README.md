color_follower
==============

Control code for a webcam-based color tracking/following robot 

This is the code to run a BeagleBone Black based robot that follows a specified color.

The hardware requirements are:
        - A frame that utilizes two direct drive motors, one on either side of the robot.
        - A webcam (low resolution is fine, this script doesn't use anything higher than 320 x 240)
        - A BeagleBone Black to run the code and drive the PWM output
        - A motor driver board (Pololu dual bidirectional motor driver is best)
        - Several power sources: 5V usb battery for BBB, 9V (or appropriate) battery for motor power

The software requirements are:
        - Angstrom Linux running the 3.8.13 kernel
        - Saad Ahmad's beaglebone-black-cpp-PWM driver modifications installed
        - Customized adafruit-beaglebone-io-python (kiorpesc's fork - for fully functional PWM)
        - opencv and python-opencv installed
