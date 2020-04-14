#!/usr/bin/env python

################################################################################
## {Description}: Image substraction ()
## Original Code from Raspberry Pi for Computer Vision - Hobbyist Bundle
## Re-use and transform into ROS nodes
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
from __future__ import division

import imutils
import cv2
import os
import imutils
import numpy as np

# import the necessary ROS packages
import sys
import rospy
import rospkg

from std_msgs.msg import Int32

rospack = rospkg.RosPack()

class ImageSubstraction_node:
	def __init__(self):
		#global pub

		# Initializing your ROS Node
		rospy.init_node('ImageSubstraction_node', anonymous=True)

		self.rate = rospy.Rate(5)

		rospy.on_shutdown(self.shutdown)

		# Publish to the image_dims topic
		#self.imgdims_pub = rospy.Publisher('image_dims', Int32, queue_size=10)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] ImageSubstraction_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

	# a function to load the input image and show its dimensions, keeping in 
	# mind that images are represented as a multi-dimensional NumPy array 
	# with shape: num rows (height) * num columns (width) * num channels (depth)
	def load(self):
		rospy.logwarn("Load Image")
		self.bg = os.path.sep.join([rospack.get_path('a_brief_tutorial_on_opencv'), 
"images", "bg.jpg"])
		self.fg = os.path.sep.join([rospack.get_path('a_brief_tutorial_on_opencv'), 
"images", "adrian.jpg"])

		self.bg = cv2.imread(self.bg)
		self.fg = cv2.imread(self.fg)

	# convert it to grayscale
	def grays(self):
		rospy.logwarn("Convert to Grayscale")

		self.graybg = cv2.cvtColor(self.bg, cv2.COLOR_BGR2GRAY)
		self.grayfg = cv2.cvtColor(self.fg, cv2.COLOR_BGR2GRAY)

	# perform background subtraction by subtracting the foreground from
	# the background and then taking the absolute value
	def bgfgsubs(self):
		rospy.logwarn("Background Substraction")

		self.sub = self.graybg.astype("int32") - self.grayfg.astype("int32")
		self.sub = np.absolute(self.sub).astype("uint8")

	# threshold the image to find regions of the subtracted image with
	# larger pixel differences
	def thresholded(self):
		rospy.logwarn("Threshold")

		self.thresh = cv2.threshold(self.sub, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

	# perform a series of erosions and dilations to remove noise
	def denoise(self):
		rospy.logwarn("De-noise")

		self.thresh = cv2.erode(self.thresh, None, iterations=1)
		self.thresh = cv2.dilate(self.thresh, None, iterations=1)

	# find contours in the thresholded difference map and then initialize
	# our bounding box regions that contains the *entire* region of motion
	def findshapes(self):
		rospy.logwarn("Find Contours")

		self.cnts = cv2.findContours(self.thresh.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = imutils.grab_contours(self.cnts)
		(self.minX, self.minY) = (np.inf, np.inf)
		(self.maxX, self.maxY) = (-np.inf, -np.inf)

		# loop over the contours
		for self.c in self.cnts:
			# compute the bounding box of the contour
			(self.x, self.y, self.w, self.h) = cv2.boundingRect(self.c)

			# reduce noise by enforcing requirements on the bounding box size
			if self.w > 20 and self.h > 20:
				# update our bookkeeping variables
				self.minX = min(self.minX, self.x)
				self.minY = min(self.minY, self.y)
				self.maxX = max(self.maxX, self.x + self.w - 1)
				self.maxY = max(self.maxY, self.y + self.h - 1)

	# show the output image and the final shape count
	def displayOutput(self):
		self.load()
		self.grays()
		self.bgfgsubs()
		self.thresholded()
		self.denoise()
		self.findshapes()

		# draw a rectangle surrounding the region of motion
		cv2.rectangle(self.fg, (self.minX, self.minY), 
				(self.maxX, self.maxY), (0, 255, 0), 2)

		cv2.imshow("Output", self.fg)
		cv2.waitKey(0)

def main(args):
	#global pub

	vn = ImageSubstraction_node()

	while not rospy.is_shutdown():
		try:
			vn.displayOutput()
			rospy.spin()

		except KeyboardInterrupt:
			rospy.loginfo("[INFO] ImageSubstraction_node [OFFLINE]...")

		cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] ImageSubstraction_node [ONLINE]...")
	main(sys.argv)
