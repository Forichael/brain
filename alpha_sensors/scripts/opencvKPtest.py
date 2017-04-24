#Figuring out Keypoints using OpenCV in python to be applied to can detection for alphabot 4.12.17
import cv2
import numpy as np
import os

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
	out[:rows1,:cols1] = img1#np.dstack([img1, img1, img1])

	# Place the next image to the right of it
	out[:rows2,cols1:] = img2#np.dstack([img2, img2, img2])

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

def isColor(image):
	#convert image into HSV, blur it
	imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	imageHSV = cv2.GaussianBlur(imageHSV, (5, 5), 0)
	#declare bounds for target pixel range
	lower_HSV = np.array([30,50,0])#([70, 100, 10])  ###Outdoor/indoor
	higher_HSV = np.array([80,255,255])#([100, 255, 255])
	#show colored pixels in range
	colorPixels = cv2.inRange(imageHSV, lower_HSV, higher_HSV)

	(cnts,_) = cv2.findContours(colorPixels.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	# try: 
	# 	#cv2.contourArea unit: pixel
	# 	colorContour = max(cnts, key = cv2.contourArea) 	
	# except:
	# 	#make a filterable exception
	# 	return [0,0]

	# if (cv2.contourArea(colorContour) > 200):
	# 		#( center (x,y), (width, height), angle of rotation )
	# 		return [cv2.minAreaRect(colorContour), colorPixels]
	# else:
	# 	return [0,0]
	cntsBig = []
	for cnt in cnts:
		if cv2.contourArea(cnt)>300:
			cntsBig.append(cnt)
	if len(cntsBig)>0:
		return True, cnts
	else:
		return False, [0,0]

def isROI(contours, frame):
	cnts_ROI=[]
	cnts_minRect=[]
	cnts_kp_ROI=[]
	cnts_des_ROI=[]
	#pulls out regions of interest for Keypoint detection
	for cnt in contours:
		try:
			minRect = cv2.minAreaRect(cnt)
			ptCen, dim, aor = minRect
			x, y = ptCen
			w, h = dim
			minRect = ((np.floor(x),np.floor(y)),(np.floor(w), np.floor(h)), aor)
			# print([np.floor(x-w/2),np.floor(x+w/2),np.floor(y-h/2),np.floor(y+h/2)])
			ROI = frame[np.floor(x-w/2):np.floor(x+w/2),np.floor(y-h/2):np.floor(y+h/2)]#region of interest
			#modify frame and use ORB to ID Keypoints
			cv2.imshow('ROI',ROI)
			kp_ROI, des_ROI = orb.detectAndCompute(ROI,None)

			cnts_ROI.append(ROI)
			cnts_minRect.append(minRect)
			cnts_kp_ROI.append(kp_ROI)
			cnts_des_ROI.append(des_ROI)

			return cnts_ROI, cnts_minRect, cnts_kp_ROI, cnts_des_ROI
		except:
			return [0,0,0], [(0,0),(0,0),0], [0], [0]

	
########### Setup ###########
#Initialize orb type descriptors
orb = cv2.ORB()
min_matches = 1 #minimum number of KP matches
num_train = 11 #number of training images
#FLANN parameters
FLANN_INDEX_KDTREE = 0;
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50) #or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params,search_params)

#Initialize video frame source
cap = cv2.VideoCapture(0)
#operating software, open control line, enter string, read and
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
	img = cv2.imread(img_path+'7up'+ str(i) +'.jpg')
	img = cv2.resize(img,dsize=(0,0), fx=0.35, fy=0.35) #make img smaller

	if img is not None:
		kp, des = orb.detectAndCompute(img, None)

		img_train.append(img)
		kp_train.append(kp)
		des_train.append(des)
	counter = 0
while(True):
	counter = counter+1
	if counter ==100000:
		counter =0
		#read the current frame from video
		ret, frame = cap.read()
		#Use HSV filter to see if there is anything the color of can
		colorBool, contours = isColor(frame)
		if colorBool and len(contours)>0:
			print('i see can color!')
			#check colored contours for Keypoints, return one with most
			cnts_ROI, cnts_minRect, cnts_kp_ROI, cnts_des_ROI = isROI(contours, frame) 
			print('i see', len(cnts_ROI), "contours")

			if len(cnts_ROI)>0:
				r_index=[]
				t_index=[]
				matches=[]
				num_matches=[]	
				print(len(cnts_ROI), num_train)
				for r in range(1, len(cnts_ROI)+1):
					frame_matches = []
					len_matches = []
					for t in range(1,num_train+1):
						print(r,t)
						good, len_good = isMatch(cnts_ROI[r-1], cnts_des_ROI[r-1], cnts_kp_ROI[r-1], \
												img_train[t-1], des_train[t-1], kp_train[t-1])
						if len_good>0:
							print('appending KP match...')
							frame_matches.append(good)
							len_matches.append(len_good)

					#Show the training image with the most keypoint matches, if any above threshhold
					if len(len_matches)>=min_matches:
						print('there are enough KP matches!', len(len_matches))
						index_best = len_matches.index(max(len_matches))
						# record_best.append([r-1, index_best, frame_matches[index_best], len_matches[index_best]])
						r_index.append(r-1)
						t_index.append(index_best)
						matches.append(frame_matches[index_best])
						num_matches.append(len_matches[index_best])	
					else:
						print('not enough KP matches.')
						r_index.append(0)
						t_index.append(0)
						matches.append(0)
						num_matches.append(0)	
				# [r_index, t_index, matches, num_matches] = record_best
				# print('TEST', r_index)
				#index of best contour (most KP matches)
				cnt_best = num_matches.index(max(num_matches)) 
				print(cnts_minRect[r_index[cnt_best]])
				box = np.int0(cv2.cv.BoxPoints(cnts_minRect[r_index[cnt_best]]))
				cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)

				cv2.imshow('frameKP', frame)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break
			else:
				break
			#if there are no keypoint matches, return largest contour
			# index_best = 0#cnts_minRect.index(max(contours, key = cv2.contourArea))

			# if (len_matches[index_best] >= min_matches):
			# 	# print('GOOD MATCH!!! :D')
			# 	# print(index_best,'---',len_matches[index_best])
			# 	match_best = drawMatches(frame,kp_frame,img_train[index_best],\
			# 						kp_train[index_best],frame_matches[index_best])
			# else:
			# 	# print('no good match')
			# 	# print(index_best,'---',len_matches[index_best])
			# 	match_best = drawMatches(frame,kp_frame,img_train[index_best],\
			# 						kp_train[index_best],[])
			# #Show Keypoints in common
			# cv2.imshow('frameKP', match_best)
			# if cv2.waitKey(1) & 0xFF == ord('q'):
			# break

		else:
			cv2.imshow('frameKP', frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break


# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()