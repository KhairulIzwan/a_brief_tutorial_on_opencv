#!/usr/bin/env python

################################################################################
## {Description}: Counting a shape (contours)
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

# import the necessary ROS packages
import sys
import rospy
import rospkg

from std_msgs.msg import Int32

rospack = rospkg.RosPack()

class CountingShape_node:
	def __init__(self):
		#global pub

		# Initializing your ROS Node
		rospy.init_node('CountingShape_node', anonymous=True)

		self.rate = rospy.Rate(5)

		rospy.on_shutdown(self.shutdown)

		# Publish to the image_dims topic
		#self.imgdims_pub = rospy.Publisher('image_dims', Int32, 
queue_size=10)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] CountingShape_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

	# a function to load the input image and show its dimensions, keeping in 
	# mind that images are represented as a multi-dimensional NumPy array 
	# with shape: num rows (height) * num columns (width) * num channels (depth)
	def load(self):
		rospy.logwarn("Load Image")
		p = os.path.sep.join([rospack.get_path('a_brief_tutorial_on_opencv'), 
"images", "shapes.png"])
		self.image = cv2.imread(p)

	# convert it to grayscale
	def grays(self):
		rospy.logwarn("Convert to Grayscale")
		self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

	# apply a Gaussian blur with a 3x3 kernel to the image to smooth it,
	# useful when reducing high frequency noise
	def blurring(self):
		rospy.logwarn("Blurring")
		self.blurred = cv2.GaussianBlur(self.gray, (3, 3), 0)

	# perform edge detection
	def edging(self):
		rospy.logwarn("Edging")
		self.edged = cv2.Canny(self.blurred, 50, 130)

	# find contours in the edge map and initialize the total number of
	# shapes found
	def count_shapes(self):
		rospy.logwarn("Find Contours")
		self.cnts = cv2.findContours(self.edged.copy(), cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)
		self.cnts = imutils.grab_contours(self.cnts)
		self.total = 0

		# loop over the contours one by one
		for self.c in self.cnts:
			# if the contour area is small, then the area is likely noise, so
			# we should ignore the contour
			if cv2.contourArea(self.c) < 25:
				continue

			# otherwise, draw the contour on the image and increment the total
			# number of shapes found
			cv2.drawContours(self.image, [self.c], -1, (204, 0, 255), 2)
			self.total += 1

	# show the output image and the final shape count
	def displayOutput(self):
		self.load()
		self.grays()
		self.blurring()
		self.edging()
		self.count_shapes()

		rospy.loginfo("Found {} shapes".format(self.total))
		cv2.imshow("Image", self.image)
		cv2.waitKey(0)

def main(args):
	#global pub

	vn = CountingShape_node()

	while not rospy.is_shutdown():
		try:
			vn.displayOutput()
			rospy.spin()

		except KeyboardInterrupt:
			rospy.loginfo("[INFO] CountingShape_node [OFFLINE]...")

		cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] CountingShape_node [ONLINE]...")
	main(sys.argv)
