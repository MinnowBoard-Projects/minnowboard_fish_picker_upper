#!/usr/bin/env python
#
# Utility based on the complete minnowboard_fish_picker-upper.py
# program, which only opens the webcam feed and attempts object
# detection, without controlling the robot arm. This can be useful
# when testing out haar paramters to account for lighting conditions
# at your setup location.
#
# TODO: A better way to do this would be to add a "debug" mode
# to the minnowboard_fish_picker-upper.py program to avoid having
# to maintain two code bases.

import sys,time

import cv2.cv as cv

# Webcam index, change if you have more than one attached USB webcam
WebcamNum = 0

# Minimum time between displaying video stream frames. You can lower the
# CPU utilization by increasing this value at the cost of choppier video.
waitkey_resolution = 50 # ms
window_title = "MinnowBoard Fish Picker-Upper"

# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=1.1, min_neighbors=3, flags=0) are tuned
# for accurate yet slow object detection. For a faster operation on real video
# images the settings are:
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING,
# min_size=<minimum possible object size>

# min_size values below 95 are prone to producing multiple object detection
# boxes for a single fish
min_size = (95, 95)
haar_scale = 1.4
min_neighbors = 5
haar_flags = 0

# Used when scaling images. 1.2 slowed down the video and allowed for
# some false positives. 2 seemed fast and accurate. If detection is not
# happening in your lighting conditions, try a value between 1.5 and 2.
# This variable helps A LOT to account for different lighting conditions:
image_scale = 1.6

haar_dbfile = "/home/root/opencv/green_fish/haarclassifier.xml"

#####################################################################

# Scan the webcam video stream for fish objects. Returns 0 if the
# time_limit (seconds to watch for) parameter was exceeded, or the X
# coordinate representing the center of the object detection box. 
def watch_for_fish(time_limit):
	global capture, cv, window_title, waitkey_resolution, time_marker

	time_marker = time.time()
	frame_copy = None
	while True:
		frame = cv.QueryFrame(capture)
		if not frame:
			cv.WaitKey(0)
			break

		# Some cameras will return frames that are vertically flipped,
		# so for code portability reasons, we create a copy of the frame
		# and flip it back if needed.
		if not frame_copy:
			# Create a frame_copy with the same parameters as frame
			frame_copy = cv.CreateImage((frame.width,frame.height),
						cv.IPL_DEPTH_8U, frame.nChannels)

		if frame.origin == cv.IPL_ORIGIN_TL:
			cv.Copy(frame, frame_copy)
		else:
			cv.Flip(frame, frame_copy, 0)

		fish_coord = detect_and_draw(frame_copy)
		if fish_coord:
			print "Fish detected at X coord ", fish_coord[0]
			return fish_coord[0]

		now = time.time()
		if now - time_marker > time_limit:
			return 0

# Based on the facedetect.py code example
def detect_and_draw(img):
	global cv, cascade, haar_scale, min_neighbors, haar_flags, min_size

	# allocate temporary images
	gray = cv.CreateImage((img.width,img.height), 8, 1)
	small_img = cv.CreateImage((cv.Round(img.width / image_scale),
					cv.Round (img.height / image_scale)), 8, 1)

	# convert color input image to grayscale
	cv.CvtColor(img, gray, cv.CV_BGR2GRAY)

	# scale input image for faster processing
	cv.Resize(gray, small_img, cv.CV_INTER_LINEAR)

	cv.EqualizeHist(small_img, small_img)

	if(cascade):
		fish = cv.HaarDetectObjects(small_img, cascade, cv.CreateMemStorage(0),
			haar_scale, min_neighbors, haar_flags, min_size)

	if fish:
		# fish is now a list of rectangles
		for ((x, y, w, h), n) in fish:
			# the input to cv.HaarDetectObjects was resized, so scale the
			# bounding box of each fish and convert it to two CvPoints
			pt1 = (int(x * image_scale), int(y * image_scale))
			pt2 = (int((x + w) * image_scale), int((y + h) * image_scale))
			print "Rectangle width is", pt2[0] - pt1[0]
			cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)

	cv.ShowImage(window_title, img)
	cv.WaitKey(waitkey_resolution)

	if fish:
		return pt1
	else:
		return False

# main:

cascade = cv.Load(haar_dbfile)
if not cascade:
	print "Error loading cascade classifier db", haar_dbfile
	exit(1)

# Capture video stream from webcam
capture = cv.CreateCameraCapture(WebcamNum)
cv.NamedWindow(window_title, 1)

while True:
	watch_for_fish(100)

cv.DestroyWindow(window_title)
