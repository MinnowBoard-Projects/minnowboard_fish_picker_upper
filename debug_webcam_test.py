#!/usr/bin/env python
#
# Simple test utility to verify we can view a video feed from the webcam
# using OpenCV.

import cv2.cv as cv

capture = cv.CreateCameraCapture(0)
cv.NamedWindow("Webcam Test", 1)

while True:
	frame = cv.QueryFrame(capture)
	if not frame:
		print "Error capturing webcam frame"
		cv.WaitKey(0)
		break
	
	cv.ShowImage("Webcam Test", frame)
	c = cv.WaitKey(35)
	# Exit on any keypress
	if c is not -1:
		break
