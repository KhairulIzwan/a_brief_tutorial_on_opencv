#!/usr/bin/env python

################################################################################
## {Description}: Detect faces (haarCascade)
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

class DetectFaces_node:
	def __init__(self):
		#global pub

		# Initializing your ROS Node
		rospy.init_node('DetectFaces_node', anonymous=True)

		self.rate = rospy.Rate(5)

		rospy.on_shutdown(self.shutdown)

		# Publish to the image_dims topic
		#self.imgdims_pub = rospy.Publisher('image_dims', Int32, queue_size=10)

	# Shutdown
	def shutdown(self):
		try:
			rospy.loginfo("[INFO] DetectFaces_node [OFFLINE]...")

		finally:
			cv2.destroyAllWindows()

	# a function to load the input image and show its dimensions, keeping in 
	# mind that images are represented as a multi-dimensional NumPy array 
	# with shape: num rows (height) * num columns (width) * num channels (depth)
	def load(self):
		rospy.logwarn("Load Image")
		p = os.path.sep.join([rospack.get_path('a_brief_tutorial_on_opencv'), 
"images", "faces_example.png"])
		self.image = cv2.imread(p)

	# convert it to grayscale
	def grays(self):
		rospy.logwarn("Convert to Grayscale")
		self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

	# load the face detector and detect faces in the image
	def detectFace(self):
		rospy.logwarn("Face Detector")

		f = os.path.sep.join([rospack.get_path('a_brief_tutorial_on_opencv'), 
"xml", "haarcascade_frontalface_default.xml"])
		self.detector = cv2.CascadeClassifier(f)
		self.rects = self.detector.detectMultiScale(self.gray, scaleFactor=1.05, 
			minNeighbors=9, minSize=(40, 40), flags=cv2.CASCADE_SCALE_IMAGE)

		rospy.loginfo("Detected {} faces".format(len(self.rects)))

	# show the output image and the final shape count
	def displayOutput(self):
		self.load()
		self.grays()
		self.detectFace()

		# loop over the bounding boxes and draw a rectangle around each face
		for (self.x, self.y, self.w, self.h) in self.rects:
			cv2.rectangle(self.image, (self.x, self.y), 
				(self.x + self.w, self.y + self.h), (0, 255, 0), 2)

		cv2.imshow("Faces", self.image)
		cv2.waitKey(0)

def main(args):
	#global pub

	vn = DetectFaces_node()

	while not rospy.is_shutdown():
		try:
			vn.displayOutput()
			rospy.spin()

		except KeyboardInterrupt:
			rospy.loginfo("[INFO] DetectFaces_node [OFFLINE]...")

		cv2.destroyAllWindows()

if __name__ == '__main__':
	rospy.loginfo("[INFO] DetectFaces_node [ONLINE]...")
	main(sys.argv)
