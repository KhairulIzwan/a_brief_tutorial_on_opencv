#!/usr/bin/env python

################################################################################
## {Description}: Loading an Image with OpenCV
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

class OpenCVBasic_node:
	def __init__(self):
		#global pub

		# Initializing your ROS Node
		rospy.init_node('OpenCVBasic_node', anonymous=True)

		self.rate = rospy.Rate(5)

		rospy.on_shutdown(self.shutdown)

		# Publish to the image_dims topic
		self.imgdims_pub = rospy.Publisher('image_dims', Int32, 
queue_size=10)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] OpenCVBasic_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

	# a function to load the input image and show its dimensions, keeping in 
	# mind that images are represented as a multi-dimensional NumPy array 
	# with shape: num rows (height) * num columns (width) * num channels (depth)
	def dimensions(self):
		p = os.path.sep.join([rospack.get_path('a_brief_tutorial_on_opencv'), 
"images", "30th_birthday.png"])
		self.image = cv2.imread(p)
		(h, w, d) = self.image.shape

		rospy.logwarn("Dimensions")
		rospy.loginfo("width={}, height={}, depth={}".format(w, h, d))

		# display the image to our screen -- we will need to click the 
		# window opened by OpenCV and press a key on our keyboard to 
		# continue execution
		cv2.imshow("Image", self.image)
		cv2.waitKey(0)

	# access the RGB pixel located at x=430, y=200, keeping in mind that
	# OpenCV stores images in BGR order rather than RGB (the pixel value
	# at this location is part of the "red" in the jeep)
	def pixelmator(self):
		(B, G, R) = self.image[200, 430]

		rospy.logwarn("Accessing Pixels")
		rospy.loginfo("R={}, G={}, B={}".format(R, G, B))

	# extract a 100x100 pixel square ROI (Region of Interest) from the
	# input image starting at x=150,y=80 and ending at x=250,y=400
	def region(self):
		self.roi = self.image[80:400, 150:250]

		rospy.logwarn("Cropping")

		cv2.imshow("ROI", self.roi)
		cv2.waitKey(0)

	# resize the image to 300x300px, ignoring aspect ratio
	def resizeNoAR(self):
		self.resized = cv2.resize(self.image, (300, 300))

		rospy.logwarn("Resizing (ignore aspect ratio)")

		cv2.imshow("Fixed Resizing", self.resized)
		cv2.waitKey(0)

	# resize the image, maintaining aspect ratio
	def resizeAR(self):
		self.resized = imutils.resize(self.image, width=300)

		rospy.logwarn("Resizing (aspect ratio)")

		cv2.imshow("Aspect Ratio Resize", self.resized)
		cv2.waitKey(0)

	# rotate the image 45 degrees clockwise
	def rotate(self):
		self.rotated = imutils.rotate(self.image, -45)

		rospy.logwarn("Rotating")

		cv2.imshow("Rotation", self.rotated)
		cv2.waitKey(0)

	# apply a Gaussian blur with a 11x11 kernel to the image to smooth it,
	# useful when reducing high frequency noise
	def blurry(self):
		self.blurred = cv2.GaussianBlur(self.image, (11, 11), 0)

		rospy.logwarn("Blurring")

		cv2.imshow("Blurred", self.blurred)
		cv2.waitKey(0)

	# draw a rectangle, circle, and line on the image, then draw text on
	# the image as well
	def textshape(self):
		cv2.rectangle(self.image, (150, 80), (250, 400), (255, 0, 255), 5)
		cv2.circle(self.image, (490, 240), 30, (255, 0, 0), -1)
		cv2.line(self.image, (0, 0), (600, 457), (0, 0, 255), 5)
		cv2.putText(self.image, "You're learning OpenCV!", (10, 435),
			cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

		rospy.logwarn("TextShape")

		cv2.imshow("Drawing", self.image)
		cv2.waitKey(0)

def main(args):
	#global pub

	vn = OpenCVBasic_node()

	while not rospy.is_shutdown():
		try:
			vn.dimensions()
			vn.pixelmator()
			vn.region()
			vn.resizeNoAR()
			vn.resizeAR()
			vn.rotate()
			vn.blurry()
			vn.textshape()

			#pub.publish(h)

			rospy.spin()

		except KeyboardInterrupt:
			rospy.loginfo("[INFO] OpenCVBasic_node [OFFLINE]...")

		cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] OpenCVBasic_node [ONLINE]...")
	main(sys.argv)
