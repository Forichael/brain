#######################TO DO:###############################
# 1. need to take calibration image using alphabot camera
# 2. pre calculate camera focal length and hardcode it into processing to save time
# 3. integrate with ROS
#
# 
#


# import the necessary packages
import numpy as np
import cv2

def distance_to_camera(knownHeight, focalLength, perHeight):
	# compute and return the distance from the maker to the camera
	return (knownHeight * focalLength) / perHeight

def find_marker(image):
	#convert image into HSV, blur it
	imageHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	imageHSV = cv2.GaussianBlur(imageHSV, (5, 5), 0)
	#declare bounds for target pixel range
	lower_pink = np.array([165, 0, 0])
	higher_pink = np.array([179, 255, 255])
	#show pink pixels in range
	pinkPixels = cv2.inRange(imageHSV, lower_pink, higher_pink)

	(cnts,_) = cv2.findContours(pinkPixels.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	try: 
		pinkContour = max(cnts, key = cv2.contourArea)
		#( center (x,y), (width, height), angle of rotation )
		return [cv2.minAreaRect(pinkContour), pinkPixels]
	except:
		pass #it breaks if there is no contour
	
def calibrate_camera(knownDistance, knownHeight, knownImage): #maybe dont have this in
	marker,_ = find_marker(cv2.imread(knownImage))
	focalLength = marker[0][2] * KNOWN_DISTANCE / KNOWN_HEIGHT
		#I think [0][2] is can height

	return focalLength

##################################START##########################################

#initialize the known (calibration) distance from the camera to the object: 24"
KNOWN_DISTANCE = 24.0
#initialize the known object height, which in this case, the can is 4.83" tall
KNOWN_HEIGHT = 4.83
#calibrate the camera for distance
# KNOWN_IMAGE = pass ########### put path to calibration image here ###############
#initialize the images aka the camera stream
cap = cv2.VideoCapture(1)
cv2.namedWindow('camera view')

#begin infinite loop
while(1):
	#read streamed frames
	_,frame = cap.read()
	(minRect, pinkPixels) = find_marker(frame)
	# distTarget = distance_to_camera(KNOWN_HEIGHT, focalLength, minRect[0][2])
	
	#obtain the info from Rect to draw a box and draw it on the frame
	box = np.int0(cv2.cv.BoxPoints(minRect))
	cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)

	# cv2.drawContours(frame, pinkContour, -1, (0,255,0), 3)	
	# cv2.imshow('camera view', pinkPixels) 
	cv2.imshow('camera view', frame) 


	k = cv2.waitKey(5) & 0xFF
	if k == 27:
		break