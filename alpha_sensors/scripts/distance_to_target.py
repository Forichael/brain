#######################TO DO:###############################
# Backlogged 1&2: --> 1a. DONE publishing center bottom pixel of can and doing ROS stuff
# 		1. need to take calibration image using alphabot camera
# 		2. pre calculate camera focal length and hardcode it into processing to save time
# 3. DONEish integrate with ROS
# 4. clean up this mess :)
# ## Figure out how to only publish frames with contours if listeners, dont do now bc info size
#


# import the necessary packages
import numpy as np
import cv2
#ROS communication
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point


def distance_to_camera(knownHeight, focalLength, perHeight):
	# compute and return the distance from the maker to the camera
	return (knownHeight * focalLength) / perHeight

def find_marker(image):
	#convert image into HSV, blur it
	imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	imageHSV = cv2.GaussianBlur(imageHSV, (5, 5), 0)
	#declare bounds for target pixel range
	lower_pink = np.array([165, 100, 10])
	higher_pink = np.array([179, 255, 255])
	#show pink pixels in range
	pinkPixels = cv2.inRange(imageHSV, lower_pink, higher_pink)

	(cnts,_) = cv2.findContours(pinkPixels.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	try: 
		#cv2.contourArea unit: pixel
		pinkContour = max(cnts, key = cv2.contourArea) 	
	except:
		#make a filterable exception
		return [0,0]

	if (cv2.contourArea(pinkContour) > 200):
			#( center (x,y), (width, height), angle of rotation )
			return [cv2.minAreaRect(pinkContour), pinkPixels]
	else:
		return [0,0]

def calibrate_camera(knownDistance, knownHeight, knownImage): #maybe dont have this in
	marker,_ = find_marker(cv2.imread(knownImage))
	focalLength = marker[0][1] * KNOWN_DISTANCE / KNOWN_HEIGHT
		#I think [0][1] is can height, 2nd term, index from 0

	return focalLength

##################################START##########################################

#initialize the known (calibration) distance from the camera to the object: 24"
KNOWN_DISTANCE = 24.0
#initialize the known object height, which in this case, the can is 4.83" tall
KNOWN_HEIGHT = 4.83
#calibrate the camera for distance
# KNOWN_IMAGE = pass ########### put path to calibration image here ###############

#initialize the images aka the camera stream
# cap = cv2.VideoCapture(1)
# cv2.namedWindow('camera view')

#initialize ROS channels
imgROS = rospy.Subscriber('/alpha/img_raw', Image)
pub = rospy.Publisher('tgt_can_bottom', Point, queue_size=10)

#begin infinite loop
while(1):
	#translate from ROS format to openCV format
	bridge = CvBridge()
	cap = bridge.imgmsg_to_cv2(imgROS, desired_encoding="bgr8")

	#read streamed frames
	_,frame = cap.read()
	(minRect, pinkPixels) = find_marker(frame)
	# distTarget = distance_to_camera(KNOWN_HEIGHT, focalLength, minRect[0][1])
	
	# inches = calibrate_camera(KNOWN_DISTANCE, KNOWN_HEIGHT, frame)
	# cv2.putText(frame, "%.2fft" % (inches / 12),
	# 	(frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
	# 	2.0, (0, 255, 0), 3)

	#if target pixels were found make a contour
	if (minRect != 0):
		#obtain the info from Rect to draw a box and draw it on the frame
		box = np.int0(cv2.cv.BoxPoints(minRect))
		cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)

		ptCen, dim,_  = minRect
		x, y = ptCen
		w, h = dim
		cv2.circle(frame, (int(x), int(y+h/2)), 10, (255, 0, 0), 3)
		
		ptCenBot = [x, y+h/2]
		#Publish the bottom centerpoint to ROS
		pub.publish(Point(x=ptCenBot[0], y=ptCenBot[1]))

	else:
		ptCen = [0,0]
		ptCenBot = [0,0]

	# cv2.drawContours(frame, pinkContour, -1, (0,255,0), 3)	
	# cv2.imshow('camera view', pinkPixels) 
##Uncomment to see, silenced for ROS:
	# cv2.imshow('camera view', frame) 


	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break