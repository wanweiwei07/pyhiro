import cv2
import numpy as np
import matplotlib.pyplot as plt

# img = cv2.imread('IMG_1560.jpg')
# gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
# edges = cv2.Canny(gray,50,150,apertureSize = 3)
# plt.imshow(edges)
# plt.show()
#
# minLineLength = 100
# maxLineGap = 10
# lines = cv2.HoughLinesP(edges,1,np.pi/180,100,minLineLength,maxLineGap)
# for x1,y1,x2,y2 in lines[0]:
#     cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
#
# plt.imshow(img)
# plt.show()




img = cv2.imread('IMG_1560.jpg')
cimg = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,1,20,
                            param1=50,param2=30,minRadius=0,maxRadius=0)

circles = np.uint16(np.around(circles))
print circles
for i in circles[0,:]:
    # draw the outer circle
    cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)


cv2.imshow('detected circles',cimg)
cv2.waitKey(0)
cv2.destroyAllWindows()