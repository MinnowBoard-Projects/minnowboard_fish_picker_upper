#!/usr/bin/env python

import sys,time
from arm_control import ArmControl

import cv2.cv as cv

# Webcam index, change if you have more than one attached USB webcam
WebcamNum = 0

# Minimum time between displaying video stream frames. You can lower the
# CPU utilization by increasing this value at the cost of choppier video.
waitkey_resolution = 50 # ms
window_title = "MinnowBoard Fish Picker-Upper"

# Amount of time it takes to completely rotate the robot arm's base
total_rotation_time = 15 # seconds (no, it's really 16.5s)
# The arm starts in the rightmost rotated position
rotation_time_left = total_rotation_time
rotation_time_right = 0
rotation_direction = ""
rotation_time_marker = 0

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
image_scale = 1.7

haar_dbfile = "/home/root/opencv/green_fish/haarclassifier.xml"

# X coordinate that represents when the arm is centered on the fish
# object. You will likely need to determine this by trail and error
# based on the alignment of your webcam's sensor and the accuracy
# you get from OpenCV's object detection. It represents what should
# be the middle of the object detection box that gets drawn around
# the object:
centered_fish_coord = 155

#####################################################################

# Grab num_frames frames from the camera, but don't bother displaying
# them. Useful to help "catch up" QueryFrame past buffered data.
def clear_camera_buffer(num_frames):
	global capture, cv

	for index in range(0, num_frames):
		frame = cv.QueryFrame(capture)

# Scan the webcam video stream for fish objects. Returns 0 if the
# time_limit (seconds to watch for) parameter was exceeded, or the X
# coordinate representing the center of the object detection box.
# You can skip the return behavior by passing False as an optional
# second argument.
def watch_for_fish(time_limit, return_when_found=True):
	global capture, cv, window_title, waitkey_resolution

	time_marker = time.time()
	frame_copy = None

	clear_camera_buffer(1)

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
		if fish_coord and return_when_found:
			#print "Fish detected at X coord ", fish_coord[0]
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
			#print "Rectangle width is", pt2[0] - pt1[0]
			cv.Rectangle(img, pt1, pt2, cv.RGB(255, 0, 0), 3, 8, 0)

	cv.ShowImage(window_title, img)
	cv.WaitKey(waitkey_resolution)

	if fish:
		return pt1
	else:
		return False

def center_on_fish():
	global centered_fish_coord, rotation_direction

	movement_steps = 0.1 # second

	while True:
		stop_base_rotation()

		clear_camera_buffer(3)
		fish_coord = watch_for_fish(3)
		if fish_coord == 0:
			# Object detection may marginally working. Nudge the arm
			# in the opposite direction to try to get us un-stuck:
			print "*** WARNING *** Timeout detecting object within center_on_fish(). Attempting recovery..."
			if rotation_direction == "right":
				rotate_base_left()
			else:
				rotate_base_right()
			time.sleep(0.1)
			continue

		# Slow down movement range if we're getting close
		if fish_coord > centered_fish_coord - 50 and fish_coord < centered_fish_coord + 50:
			print "Reducing base rotation steps to 0.05s"
			movement_steps = 0.05

		# Aiming for within 5 pixels of our target
		if fish_coord < centered_fish_coord - 3:
			rotate_base_left()
			time.sleep(movement_steps)
		elif fish_coord > centered_fish_coord + 3:
			rotate_base_right()
			time.sleep(movement_steps)
		else:
			print "Centered! Final fish_coord is", fish_coord
			return

def rotate_base_left():
	print "Rotating base to the left"
	global rotation_time_marker, rotation_direction
	rotation_direction = "left"
	rotation_time_marker = time.time()
	cmd = arm.buildcommand(0,0,0,0,2)
	arm.sendcommand(dev,cmd)

def rotate_base_right():
	print "Rotating base to the right"
	global rotation_time_marker, rotation_direction
	rotation_direction = "right"
	rotation_time_marker = time.time()
	cmd = arm.buildcommand(0,0,0,0,1)
	arm.sendcommand(dev,cmd)

def stop_base_rotation():
	print "Stopping base rotation"
	global rotation_time_marker, rotation_time_left, rotation_time_right
	arm.sendcommand(dev)

	now = time.time()
	elapsed_time = now - rotation_time_marker

	if elapsed_time > total_rotation_time:
		# This can happen when retrying pick-ups, so don't update
		# the rotation time variables
		return

	if rotation_direction == "left":
		rotation_time_left = rotation_time_left - elapsed_time
	else:
		rotation_time_left = rotation_time_left + elapsed_time
	rotation_time_right = total_rotation_time - rotation_time_left
	watch_for_fish(1)

# Timing values produced by trial and error - these worked for picking
# up a fish located six inches from the outer edge of the OWI robot base.
def pick_up():
	print "Picking up fish"

	# Don't run watch_for_fish() immediately so the last object
	# detection ROI appears on the screen, and the operator can
	# explain how its accuracy might impact the success of this
	# picking up operation.

	# Elbow down
	cmd = arm.buildcommand(0,2,0,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(4.1)
	arm.sendcommand(dev)

	# Wrist up
	cmd = arm.buildcommand(0,0,1,0,0)
	arm.sendcommand(dev,cmd)
	time.sleep(3.25)
	arm.sendcommand(dev)

	# Open grip
	cmd = arm.buildcommand(0,0,0,2,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(1.75, False)
	arm.sendcommand(dev)

	# Shoulder down
	cmd = arm.buildcommand(2,0,0,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(1.4, False)
	arm.sendcommand(dev)

	# Close grip
	cmd = arm.buildcommand(0,0,0,1,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(1.3, False)
	arm.sendcommand(dev)

	# Elbow up
	cmd = arm.buildcommand(0,1,0,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(4.45, False)
	arm.sendcommand(dev)

	watch_for_fish(0.5, False)

def undo_pick_up():
	print "Undo-ing pick up"

	# Elbow up
	cmd = arm.buildcommand(0,1,0,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(0.45, False)
	arm.sendcommand(dev)

	# Close grip
	cmd = arm.buildcommand(0,0,0,1,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(0.5, False)
	arm.sendcommand(dev)

	# Wrist down
	cmd = arm.buildcommand(0,0,2,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(3.2, False)
	arm.sendcommand(dev)

	# Shoulder up
	cmd = arm.buildcommand(1,0,0,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(1.8, False)
	arm.sendcommand(dev)

def move_to_plate():
	global rotation_time_left

	rotate_base_left()
	watch_for_fish(rotation_time_left, False)
	stop_base_rotation()

def put_down():
	print "Putting down fish"

	# Elbow down
	cmd = arm.buildcommand(0,2,0,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(3.5, False)
	arm.sendcommand(dev)

	# Open grip
	cmd = arm.buildcommand(0,0,0,2,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(1.25, False)
	arm.sendcommand(dev)

def return_to_calibration_position():
	global rotation_time_right

	print "Returning to calibration position"

	# Shoulder up
	cmd = arm.buildcommand(1,0,0,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(1.8, False)
	arm.sendcommand(dev)

	# Elbow up
	cmd = arm.buildcommand(0,1,0,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(4.65, False)
	arm.sendcommand(dev)

	# Close grip
	cmd = arm.buildcommand(0,0,0,1,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(1.6, False)
	arm.sendcommand(dev)

	# Wrist down
	cmd = arm.buildcommand(0,0,2,0,0)
	arm.sendcommand(dev,cmd)
	watch_for_fish(3.2, False)
	arm.sendcommand(dev)

	# Base clockwise
	rotate_base_right()
	watch_for_fish(rotation_time_right - 0.6, False)
	stop_base_rotation()

def pick_up_fish():
	center_on_fish()
	watch_for_fish(0.5, False)
	pick_up()
	fish_coord = watch_for_fish(1)
	if fish_coord > 0:
		# Pick up attempt failed, so retry
		undo_pick_up()
		pick_up_fish()
		
# main:

# OWI robot arm setup
arm = ArmControl()
dev = arm.connecttoarm()

cascade = cv.Load(haar_dbfile)
if not cascade:
	print "Error loading cascade classifier db", haar_dbfile
	exit(1)

# Capture video stream from webcam
capture = cv.CreateCameraCapture(WebcamNum)
cv.NamedWindow(window_title, 1)

# Ensure the video stream is visible before starting base rotation
watch_for_fish(1)

# I should really use one of the GPIO libraries for this, but
# it's late and I need to demo this in the morning:
# GPIO pin 5 corresponds to gpio246                    
gpio_direction_fn = "/sys/class/gpio/gpio246/direction"
gpio_value_fn = "/sys/class/gpio/gpio246/value"
                                           
# Set up GPIO pin as an input:             
direction_fd = open(gpio_direction_fn, 'w')
direction_fd.write("in")

while True:
	value_fd = open(gpio_value_fn, 'r')
	val = value_fd.readline()
	if val == "1\n":
		break
	value_fd.close()
	watch_for_fish(0.05)

while True:
	rotate_base_left()
	fish_coord = watch_for_fish(rotation_time_left)
	if fish_coord > 0:
		pick_up_fish()
		move_to_plate()
		put_down()
		return_to_calibration_position()
		break

	stop_base_rotation()
	rotate_base_right()
	fish_coord = watch_for_fish(rotation_time_right)
	if fish_coord > 0:
		pick_up_fish()
		move_to_plate()
		put_down()
		return_to_calibration_position()
		break
	stop_base_rotation()

cv.DestroyWindow(window_title)
