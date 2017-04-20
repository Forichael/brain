#Figuring out Keypoints using OpenCV in python to be applied to can detection for alphabot 4.12.17
import cv2
import numpy as np
import os

# #ROS communication
# import rospy
# import tf
# import roslib
# from sensor_msgs.msg import Image


### drawMatches() not defined until opencv3.0.0
def drawMatches(img1, kp1, img2, kp2, matches):
	"""
	http://stackoverflow.com/questions/20259025/module-object-has-no-attribute-drawmatches-opencv-python

	My own implementation of cv2.drawMatches as OpenCV 2.4.9
	does not have this function available but it's supported in
	OpenCV 3.0.0

	This function takes in two images with their associated 
	keypoints, as well as a list of DMatch data structure (matches) 
	that contains which keypoints matched in which images.

	An image will be produced where a montage is shown with
	the first image followed by the second image beside it.

	Keypoints are delineated with circles, while lines are connected
	between matching keypoints.

	img1,img2 - Grayscale images
	kp1,kp2 - Detected list of keypoints through any of the OpenCV keypoint 
			  detection algorithms
	matches - A list of matches of corresponding keypoints through any
			  OpenCV keypoint matching algorithm
	"""

	# Create a new output image that concatenates the two images together
	# (a.k.a) a montage
	rows1 = img1.shape[0]
	cols1 = img1.shape[1]
	rows2 = img2.shape[0]
	cols2 = img2.shape[1]

	out = np.zeros((max([rows1,rows2]),cols1+cols2,3), dtype='uint8')

	# Place the first image to the left
	out[:rows1,:cols1] = np.dstack([img1, img1, img1])

	# Place the next image to the right of it
	out[:rows2,cols1:] = np.dstack([img2, img2, img2])

	# For each pair of points we have between both images
	# draw circles, then connect a line between them
	for mat in matches:

		# Get the matching keypoints for each of the images
		img1_idx = mat.queryIdx
		img2_idx = mat.trainIdx

		# x - columns
		# y - rows
		(x1,y1) = kp1[img1_idx].pt
		(x2,y2) = kp2[img2_idx].pt

		# Draw a small circle at both co-ordinates
		# radius 4
		# colour blue
		# thickness = 1
		cv2.circle(out, (int(x1),int(y1)), 4, (255, 0, 0), 1)   
		cv2.circle(out, (int(x2)+cols1,int(y2)), 4, (255, 0, 0), 1)

		# Draw a line in between the two points
		# thickness = 1
		# colour blue
		cv2.line(out, (int(x1),int(y1)), (int(x2)+cols1,int(y2)), (255, 0, 0), 1)

	# Show the image
	# cv2.imshow('Matched Features', out)
	# cv2.waitKey(0)
	# cv2.destroyWindow('Matched Features')

	# Also return the image if you'd like a copy
	return out	

def isMatch(img1, des1, kp1, img2, des2, kp2):
		#Make sure that the descriptors are correct format for knnMatch
		#checks the type of array, and recasts array as type float32 if not already
		try:
			if(des1.dtype != np.float32):
				des1 = des1.astype(np.float32)
			if(des2.dtype != np.float32):
				des2 = des2.astype(np.float32)

			matches = flann.knnMatch(des1, des2, k=2)

			#Filter for good matches
			good = []
			dist = []
			# ratio test as per Lowe's paper
			for i,(m,n) in enumerate(matches):
				if m.distance < 0.7*n.distance:
					good.append(m)
					# dist.append(n)
			return good, len(good)
				
		except:
			return [], 0
	
### Setup
#Initialize orb type descriptors
orb = cv2.ORB()
min_matches = 5 #minimum number of KP matches
num_train = 11 #number of training images
#FLANN matching using ORB descriptors
#FLANN parameters
FLANN_INDEX_KDTREE = 0;
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50) #or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params,search_params)

#Initialize video frame source
cap = cv2.VideoCapture(0)
#operating software, open control line, enter string, read  and
# return results, remove leading or trailing whitespace
sensor_dir = os.popen('rospack find alpha_sensors').read().strip()
#Initialize training images
img_path = sensor_dir+'/data/7upCanCrop/'

img_train = []
kp_train = []
des_train = []

#fill the training lists using the training images
for i in range(1,num_train+1):
	# print(img_path+'7up'+ str(i) +'.jpg')
	img =cv2.imread(img_path+'7up'+ str(i) +'.jpg')
	# cv2.imshow('img',img)
	gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
	kp, des = orb.detectAndCompute(gray, None)

	img_train.append(gray)
	kp_train.append(kp)
	des_train.append(des)
	
	# print(img_train)
	# print('-------')
	# print(des_train)
	# cv2.imshow('Training Images', img_train[i-1])

	counter = 0
while(True):
	counter=counter+1
	if(counter==1000):
		counter=0
		#read the current frame from video
		ret, frame = cap.read()
		#modify frame and use ORB to ID Keypoints
		gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		kp_frame, des_frame = orb.detectAndCompute(gray_frame,None)

		frame_matches = []
		len_matches = []
		for i in range(1,num_train+1):
			good, len_good = isMatch(gray_frame, des_frame, kp_frame, \
										img_train[i-1], des_train[i-1], kp_train[i-1])
			frame_matches.append(good)
			len_matches.append(len_good)
		
		#Show the training image with the most keypoint matches, if any above threshhold
		index_best = len_matches.index(max(len_matches))

		if (len_matches[index_best] >= min_matches):
			# print('GOOD MATCH!!! :D')
			# print(index_best,'---',len_matches[index_best])
			match_best = drawMatches(gray_frame,kp_frame,img_train[index_best],\
								kp_train[index_best],frame_matches[index_best])
		else:
			# print('no good match')
			# print(index_best,'---',len_matches[index_best])
			match_best = drawMatches(gray_frame,kp_frame,img_train[index_best],\
								kp_train[index_best],[])
		#Show Keypoints in common
		cv2.imshow('frameKP', match_best)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()