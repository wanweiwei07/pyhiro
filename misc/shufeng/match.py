import numpy as np
import cv2
from matplotlib import pyplot as plt

timg = cv2.imread('template.png')
timggray= cv2.cvtColor(timg,cv2.COLOR_BGR2GRAY)
qimg = cv2.imread('IMG_1561.jpg')
qimggray= cv2.cvtColor(qimg,cv2.COLOR_BGR2GRAY)

# Initiate ORB detector
orb = cv2.ORB_create()

# find the keypoints and descriptors with ORB
kp1, des1 = orb.detectAndCompute(timggray, None)
kp2, des2 = orb.detectAndCompute(qimggray, None)

# create BFMatcher object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors.
matches = bf.match(des1,des2)

# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)
print matches
# Draw first 10 matches.
img3 = cv2.drawMatches(timggray,kp1,qimggray,kp2,matches ,None, flags=2)

plt.imshow(img3),plt.show()