#Human visualization of contour and keypoints that alphabot sees

# import the necessary packages
import numpy as np
import cv2
import os

def find_marker(image):
    #convert image into HSV, blur it
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    imageHSV = cv2.GaussianBlur(imageHSV, (9, 9), 0)
    #declare bounds for target pixel range
    lower_HSV = np.array([55, 100, 0]) #([70, 100, 10])  ###Outdoor/indoor 7up can
    higher_HSV = np.array([70, 255, 255]) #([100, 255, 255])
    #show color pixels in range
    colorPixels = cv2.inRange(imageHSV, lower_HSV, higher_HSV)

    (cnts,_) = cv2.findContours(colorPixels.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    try: 
        #cv2.contourArea unit: pixel
        colorContour = max(cnts, key = cv2.contourArea)     
        if (cv2.contourArea(colorContour) > 100):
            # print "Can color found"
            # print(cv2.boundingRect(colorContour))
            return cv2.boundingRect(colorContour), colorPixels
            #return cv2.minAreaRect(colorContour), colorPixels
        else:
            # print "No can color found (too small)"
            return 0,0
    except:
        #make a filterable exception
        #print "No can found at all"
        return 0,0

def find_KPmatches(img1, des1, kp1, img2, des2, kp2):
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
            # dist = []
            # ratio test as per Lowe's paper
            for i,(m,n) in enumerate(matches):
                if m.distance < 0.7*n.distance:
                    good.append(m)
                    # dist.append(n)
            return good, len(good)
                
        except:
            return [], 0
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

##################################START##########################################

#Initialize orb type descriptors
orb = cv2.ORB()
min_matches = 1 #minimum number of KP matches
num_train = 11 #number of training images
#FLANN parameters
FLANN_INDEX_KDTREE = 0;
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50) #or pass empty dictionary
flann = cv2.FlannBasedMatcher(index_params,search_params)
minReqMatches = 6 #Minimum amount of KP matches to ID can
#Initialize training image info
# operating software, open control line, enter string, read 
# and return results, remove leading or trailing whitespace
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

#initialize the images aka the camera stream
cap = cv2.VideoCapture(1)
cv2.namedWindow('camera view')

def img_cb():
    _,frame = cap.read()

    cv2.imshow('frame', frame)
    # bridge = CvBridge()
    # frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    result = find_marker(frame)


    if result is not None:
        minRect, colorPixels = result
    #cv2.imshow('color', colorPixels)
    #cv2.waitKey(10)

    x,y,z = 0,0,0

    if (minRect != 0):
        #obtain the info from Rect to draw a box and draw it on the frame
        #box = np.int0(cv2.cv.BoxPoints(minRect))
        #cv2.drawContours(frame, [box], -1, (0, 255, 0), 2)
        # print minRect
        x,y,w,h  = minRect
        cv2.circle(frame, (int(x), int(y+h/2)), 10, (255, 0, 0), 3)

        ptCenBot = [x, y+h/2]

        #check for keypoints inside the box (region of interest)
        
        # ROI = frame[np.floor(x-w/2):np.floor(x+w/2),np.floor(y-h/2):np.floor(y+h/2)]
        ROI = np.zeros(frame.shape,np.uint8)
        ROI[y:y+h,x:x+w] = frame[y:y+h,x:x+w]

        kp_ROI, des_ROI = orb.detectAndCompute(ROI,None)
        bestMatches = []
        bestNumMatches = 0
        bestTrainImg = 0
        for t in range(1, num_train):
            matches, numMatches = find_KPmatches(ROI, des_ROI, kp_ROI, \
                                        img_train[t], des_train[t], kp_train[t])
            #only hold onto the best match, if any
            if numMatches > bestNumMatches:
                bestNumMatches = numMatches #number of KP matches made
                bestMatches = matches #list of matched points
                bestTrainImg = t #index of the best matched training image

        # Could publish number of matches associated with the best contour,
        #   if less than threshhold notes location and keeps looking
        # For human viewing need matches, ROI, img_train[bestTrainingImg]
        #   and the respective keypoints. If not ROI then minRect.

        if bestNumMatches >= minReqMatches:
            print(bestNumMatches)
            visMatches = drawMatches(ROI, kp_ROI, img_train[bestTrainImg], \
                            kp_train[bestTrainImg], bestMatches)

        else:
            visMatches = drawMatches(ROI, kp_ROI, img_train[0], \
                            kp_train[0], [])
        return visMatches
        
    else:
        return frame

        #Publish the bottom centerpoint to ROS
#        cv2.imshow('frame', frame) 
#        cv2.imshow('can', colorPixels) 
#        cv2.waitKey(10)
##Uncomment to see, silenced for ROS:

def main():
    while(1):
        thing= img_cb()
        cv2.imshow('camera view', thing) 

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break
    # global tgt_pub
    # #initialize ROS channels
    # rospy.init_node('can_finder')
    # img_sub = rospy.Subscriber('/alpha/image_raw', Image, img_cb)
    # tgt_pub = rospy.Publisher('can_point', PointStamped, queue_size=10)
    # # kp_pub = rospy.Publisher('kp_can_info', Type, queue_size=10)

    # #rate = rospy.Rate(20)
    # rospy.spin()

    #begin infinite loop
    #while not rospy.is_shutdown():
    #    #translate from ROS format to openCV format
    #    bridge = CvBridge()
    #    cap = bridge.imgmsg_to_cv2(imgROS, desired_encoding="bgr8")

    #    #read streamed frames
    #    _,frame = cap.read()
    #    (minRect, colorPixels) = find_marker(frame)
    #    # distTarget = distance_to_camera(KNOWN_HEIGHT, focalLength, minRect[0][1])
    #    #if target pixels were found make a contour

    #    #k = cv2.waitKey(5) & 0xFF
    #    #if k == 27:
    #    #        break

if __name__ == "__main__":
    main()