#!/usr/bin/env python

################################################################################
## {Description}: Accessing raspicam/usbcam
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import Bool
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CameraInfo

#from cv_bridge import CvBridge
#from cv_bridge import CvBridgeError

import rospy

class ControlLogic:
	def __init__(self):

#		self.bridge = CvBridge()
#		self.image_received = False
		
		self.pump0 = Bool()
		self.pump1 = Bool()
		self.pump2 = Bool()
		self.pump3 = Bool()

		rospy.logwarn("ControlLogic Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

#		# Subscribe to Image msg
#		self.image_topic = "/cv_camera_robot1/image_raw"
##		self.image_topic = "/camera/rgb/image_raw"
#		self.image_sub = rospy.Subscriber(self.image_topic, Image, self.cbImage)

#		# Subscribe to CameraInfo msg
#		self.cameraInfo_topic = "/cv_camera_robot1/camera_info"
##		self.cameraInfo_topic = "/camera/rgb/camera_info"
#		self.cameraInfo_sub = rospy.Subscriber(self.cameraInfo_topic, CameraInfo, 
#			self.cbCameraInfo)

		# Subscribe to val_Temp msg
		self.valTemp_topic = "/val_Temp"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valTemp_sub = rospy.Subscriber(self.valTemp_topic, Float64, 
			self.cbvalTemp)

		# Subscribe to val_Humid msg
		self.valHumid_topic = "/val_Humid"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valHumid_sub = rospy.Subscriber(self.valHumid_topic, Float64, 
			self.cbvalHumid)

		# Subscribe to val_soil0 msg
		self.val_soil0_topic = "/val_soil0"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valSoil0_sub = rospy.Subscriber(self.val_soil0_topic, Int64, 
			self.cbvalSoil0)
			
		# Publish to pump0 msg
		self.val_pump0_topic = "/pump0"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valpump0_pub = rospy.Publisher(self.val_pump0_topic, Bool,queue_size=10)

		# Subscribe to val_soil1 msg
		self.val_soil1_topic = "/val_soil1"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valSoil1_sub = rospy.Subscriber(self.val_soil1_topic, Int64, 
			self.cbvalSoil1)

		# Publish to pump1 msg
		self.val_pump1_topic = "/pump1"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valpump1_pub = rospy.Publisher(self.val_pump1_topic, Bool,queue_size=10)
		
		# Subscribe to val_soil2 msg
		self.val_soil2_topic = "/val_soil2"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valSoil2_sub = rospy.Subscriber(self.val_soil2_topic, Int64, 
			self.cbvalSoil2)
			
		# Publish to pump2 msg
		self.val_pump2_topic = "/pump2"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valpump2_pub = rospy.Publisher(self.val_pump2_topic, Bool,queue_size=10)

		# Subscribe to val_soil0 msg
		self.val_soil3_topic = "/val_soil3"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valSoil3_sub = rospy.Subscriber(self.val_soil3_topic, Int64, 
			self.cbvalSoil3)

		# Publish to pump3 msg
		self.val_pump3_topic = "/pump3"
#		self.cameraInfo_topic = "/camera/rgb/camera_info"
		self.valpump3_pub = rospy.Publisher(self.val_pump3_topic, Bool,queue_size=10)
		
		# Allow up to one second to connection
		rospy.sleep(2)

#	# Convert image to OpenCV format
#	def cbImage(self, msg):

#		try:
#			self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#			# comment if the image is mirrored
##			self.cv_image = cv2.flip(self.cv_image, 1)
#		except CvBridgeError as e:
#			print(e)

#		if self.cv_image is not None:
#			self.image_received = True
#		else:
#			self.image_received = False

#	# Get CameraInfo
#	def cbCameraInfo(self, msg):

#		self.imgWidth = msg.width
#		self.imgHeight = msg.height

	# Get valTemp
	def cbvalTemp(self, msg):

		self.valTemp = msg.data

	# Get valHumid
	def cbvalHumid(self, msg):

		self.valHumid = msg.data

	# Get valSoil0
	def cbvalSoil0(self, msg):

		self.valSoil0 = msg.data

	# Get valSoil1
	def cbvalSoil1(self, msg):

		self.valSoil1 = msg.data

	# Get valSoil2
	def cbvalSoil2(self, msg):

		self.valSoil2 = msg.data

	# Get valSoil3
	def cbvalSoil3(self, msg):

		self.valSoil3 = msg.data


#	# Image information callback
#	def cbInfo(self):

#		fontFace = cv2.FONT_HERSHEY_DUPLEX
#		fontScale = 0.7
#		color = (255, 255, 255)
#		thickness = 1
#		lineType = cv2.LINE_AA
#		bottomLeftOrigin = False # if True (text upside down)

#		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

#		cv2.putText(self.cv_image, "{}".format(self.timestr), (10, 20), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Temperature (C): %.2f" % (self.valTemp), (10, 40), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Humidity (%%): %.2f" % (self.valHumid), (10, 60), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Soil0: %d" % (self.valSoil0), (10, 80), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Soil1: %d" % (self.valSoil1), (10, 100), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Soil2: %d" % (self.valSoil2), (10, 120), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Soil3: %d" % (self.valSoil3), (10, 140), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "Sample", (10, self.imgHeight-10), 
#			fontFace, fontScale, color, thickness, lineType, 
#			bottomLeftOrigin)
#		cv2.putText(self.cv_image, "(%d, %d)" % (self.imgWidth, self.imgHeight), 
#			(self.imgWidth-100, self.imgHeight-10), fontFace, fontScale, 
#			color, thickness, lineType, bottomLeftOrigin)

#	# Show the output frame
#	def cbShowImage(self):
#		self.cv_image_clone = imutils.resize(
#						self.cv_image.copy(),
#						width=1920
#						)

#		cv2.imshow("ControlLogic", self.cv_image_clone)
#		cv2.waitKey(1)

	# Preview image + info
	def cbControlLogic(self):

		if self.valSoil0 > 800:
			self.pump0.data = True
		else:
			self.pump0.data = False
			
		if self.valSoil1 > 800:
			self.pump1.data = True
		else:
			self.pump1.data = False
			
		if self.valSoil2 > 800:
			self.pump2.data = True
		else:
			self.pump2.data = False
			
		if self.valSoil3 > 800:
			self.pump3.data = True
		else:
			self.pump3.data = False
			
		self.valpump0_pub.publish(self.pump0)
		self.valpump1_pub.publish(self.pump1)
		self.valpump2_pub.publish(self.pump2)
		self.valpump3_pub.publish(self.pump3)

	# rospy shutdown callback
	def cbShutdown(self):
	
		self.pump0.data=False
		self.pump1.data=False
		self.pump2.data=False
		self.pump3.data=False
		
		self.valpump0_pub.publish(self.pump0)
		self.valpump1_pub.publish(self.pump1)
		self.valpump2_pub.publish(self.pump2)
		self.valpump3_pub.publish(self.pump3)
		
		rospy.logerr("ControlLogic Node [OFFLINE]...")
#		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('control_logic', anonymous=False)
	cl = ControlLogic()
	
	r = rospy.Rate(10)

	# Camera preview
	while not rospy.is_shutdown():
		cl.cbControlLogic()
		r.sleep()
