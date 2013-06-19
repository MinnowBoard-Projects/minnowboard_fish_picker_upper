#!/usr/bin/env python

import sys,time
from arm_control import ArmControl

import cv2.cv as cv

# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned 
# for accurate yet slow object detection. For a faster operation on real video 
# images the settings are: 
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING, 
# min_size=<minimum possible object size>
min_size = (60, 60)
image_scale = 2
haar_scale = 1.2
min_neighbors = 8
haar_flags = 0

haar_dbfile = "/home/root/opencv/green_fish/haarclassifier.xml"

# X coordinate that represents when the arm is centered on the fish
# object. You will likely need to determine this by trail and error
# based on the alignment of your webcam's sensor and the accuracy
# you get from OpenCV's object detection. It represents what should
# be the middle of the object detection box that gets drawn around
# the object:
centered_fish_coord = 165

# Change this if you have more than one attached USB webcam
WebcamNum = 0

total_rotation_time = 16.5 # seconds
base_rotation_time = total_rotation_time
rotation_direction = ""
time_marker = 0

#####################################################################

# Based on the facedetect.py code example
def detect_and_draw(img, cascade):
	global capture, cv

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
			cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)

	cv.ShowImage("MinnowBoard Fish Picker-Upper", img)
	cv.WaitKey(2)

	if fish:
		#print "Found a fish! - %d x %d" % pt1
		return pt1
	else:
		return False

# Return the coordinates of a reliable fish detection. Does not return
# until a detection occurs!
def watch_for_fish():
	global capture, cv, cascade

	detect_counter = 0
	# min number of consecutive object detect frames before we go to
	# centering state
	detect_counter_threshold = 2
	
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

		fish_coord = detect_and_draw(frame_copy, cascade)
		if fish_coord:
			detect_counter = detect_counter + 1
		else:
			detect_counter = 0

		if detect_counter > detect_counter_threshold:
			return fish_coord[0]

def scan_for_fish_state():
	print "Starting left rotation, scanning for fish"
	rotate_base_left()
	watch_for_fish()
	center_on_fish_state()

def center_on_fish_state():
	print "Centering on fish"
	global centered_fish_coord

	movement_steps = 0.1 # second

	while True:
		stop_base_rotation()

		fish_coord = watch_for_fish()
		print "Current fish_coord:", fish_coord

		# Slow down movement range if we're getting close
		if fish_coord > centered_fish_coord - 50 and fish_coord < centered_fish_coord + 50:
			print "Reducing base rotation steps to 0.05s"
			movement_steps = 0.05

		# Aiming for within 5 pixels of our target
		if fish_coord < centered_fish_coord - 2:
			rotate_base_left()
			time.sleep(movement_steps)
		elif fish_coord > centered_fish_coord + 2:
			rotate_base_right()
			time.sleep(movement_steps)
		else:
			print "Centered! Final fish_coord is", fish_coord
			return

def rotate_base_left():
	print "Rotating base to the left"
	global time_marker, rotation_direction
	rotation_direction = "left"
	time_marker = time.time()
	cmd = arm.buildcommand(0,0,0,0,2)
	arm.sendcommand(dev,cmd)

def rotate_base_right():
	print "Rotating base to the right"
	global time_marker, rotation_direction
	rotation_direction = "right"
	time_marker = time.time()
	cmd = arm.buildcommand(0,0,0,0,1)
	arm.sendcommand(dev,cmd)

def stop_base_rotation():
	print "Stopping base rotation"
	global time_marker, base_rotation_time
	arm.sendcommand(dev)

	now = time.time()
	elapsed_time = now - time_marker
	if rotation_direction == "left":
		base_rotation_time = base_rotation_time - elapsed_time
	else:
		base_rotation_time = base_rotation_time + elapsed_time
	time.sleep(0.5)

# Timing values produced by trial and error - these worked for picking
# up a fish located six inches from the outer edge of the robot base.
def pick_up():
	print "Picking up fish"

	# Elbow down
	cmd = arm.buildcommand(0,2,0,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(4.0)
	arm.sendcommand(dev)

	# Wrist up
	cmd = arm.buildcommand(0,0,1,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(3.25)
	arm.sendcommand(dev)

	# Open grip
	cmd = arm.buildcommand(0,0,0,2,0)
	arm.sendcommand(dev,cmd)
	time.sleep(1.8)
	arm.sendcommand(dev)

	# Shoulder down
	cmd = arm.buildcommand(2,0,0,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(1.4)
	arm.sendcommand(dev)

	# Close grip
	cmd = arm.buildcommand(0,0,0,1,0)
	arm.sendcommand(dev,cmd)
	time.sleep(1.3)
	arm.sendcommand(dev)

	# Elbow up
	cmd = arm.buildcommand(0,1,0,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(4.7)
	arm.sendcommand(dev)

def move_to_plate():
	global base_rotation_time
	if base_rotation_time < 0:
		print "Error: base_rotation_time is", base_rotation_time

	rotate_base_left()
	time.sleep(base_rotation_time)
	stop_base_rotation()

def put_down():
	print "Putting down fish"

	# Elbow down
	cmd = arm.buildcommand(0,2,0,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(3.5)
	arm.sendcommand(dev)

	# Open grip
	cmd = arm.buildcommand(0,0,0,2,0)
	arm.sendcommand(dev,cmd)
	time.sleep(1.3)
	arm.sendcommand(dev)

def return_to_calibration_position():
	global total_rotation_time, base_rotation_time

	print "Returning to calibration position"

	# Shoulder up
	cmd = arm.buildcommand(1,0,0,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(1.8)
	arm.sendcommand(dev)

	# Elbow up
	cmd = arm.buildcommand(0,1,0,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(4.35)
	arm.sendcommand(dev)

	# Close grip
	cmd = arm.buildcommand(0,0,0,1,0)
	arm.sendcommand(dev,cmd)
	time.sleep(1.75)
	arm.sendcommand(dev)

	# Wrist down
	cmd = arm.buildcommand(0,0,2,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(3.1)
	arm.sendcommand(dev)

	# Base clockwise
	rotate_base_right()
	time.sleep(total_rotation_time - 0.8)
	stop_base_rotation()
	base_rotation_time = total_rotation_time

# main:

debug = 0
if len(sys.argv) != 1 and sys.argv[1] == "debug":
	debug = 1

# OWI robot arm setup
arm = ArmControl()
dev = arm.connecttoarm()

cascade = cv.Load(haar_dbfile)
if not cascade:
	print "Error loading cascade classifier db", haar_dbfile
	exit()

# Capture video stream from webcam
capture = cv.CreateCameraCapture(WebcamNum)
cv.NamedWindow("MinnowBoard Fish Picker-Upper", 1)

if capture:
	if debug:
		while True:
			watch_for_fish()
	else:
		# state machine:
		scan_for_fish_state()
		pick_up()
		move_to_plate()
		put_down()
		return_to_calibration_position()

	exit()
else:
	print "Error capturing video from webcam", WebcamNum
	exit()

cv.DestroyWindow("MinnowBoard Fish Picker-Upper")
