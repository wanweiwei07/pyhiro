import cv2
import numpy as np
from matplotlib import pyplot as plt

def checkAvailability(sift, tkp, tdes, matchimg):
    """

    :param sift:
    :param tkp:
    :param tdes:sift feature object, template keypoints, and template descriptor
    :param matchimg:
    :return:
    """

    qimg = cv2.imread(matchimg)
    qimggray = cv2.cvtColor(qimg,cv2.COLOR_BGR2GRAY)
    # kernel = np.ones((5,5), np.uint8)
    # qimggray = cv2.erode(qimggray, kernel, iterations=1)
    # ret,threshimg = cv2.threshold(qimggray,100,255,cv2.THRESH_BINARY)
    qkp,qdes = sift.detectAndCompute(qimggray, None)
    # plt.imshow(threshimg, 'gray'), plt.show()

    FLANN_INDEX_KDITREE=0
    index_params=dict(algorithm=FLANN_INDEX_KDITREE,tree=5)
    # FLANN_INDEX_LSH = 6
    # index_params = dict(algorithm=FLANN_INDEX_LSH,
    #                     table_number=12,  # 12
    #                     key_size=20,  # 20
    #                     multi_probe_level=2)  # 2
    search_params = dict(checks = 50)
    flann=cv2.FlannBasedMatcher(index_params,search_params)
    matches=flann.knnMatch(tdes,qdes,k=2)
    goodMatch=[]
    for m_n in matches:
        if len(m_n) != 2:
            continue
        m, n = m_n
        if(m.distance<0.75*n.distance):
            goodMatch.append(m)
    MIN_MATCH_COUNT = 30
    if (len(goodMatch) >= MIN_MATCH_COUNT):
        tp = []
        qp = []

        for m in goodMatch:
            tp.append(tkp[m.queryIdx].pt)
            qp.append(qkp[m.trainIdx].pt)

        tp, qp = np.float32((tp, qp))
        H, status = cv2.findHomography(tp, qp, cv2.RANSAC, 3.0)

        h = timg.shape[0]
        w = timg.shape[1]
        trainBorder = np.float32([[[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]])
        queryBorder = cv2.perspectiveTransform(trainBorder, H)
        cv2.polylines(qimg, [np.int32(queryBorder)], True, (0, 255, 0), 5)
        cv2.imshow('result', qimg)
        plt.imshow(qimg, 'gray'), plt.show()
        return True
    else:
        print "Not Enough match found- %d/%d" % (len(goodMatch), MIN_MATCH_COUNT)
        return False
    # cv2.imshow('result', qimg)
    # if cv2.waitKey(10) == ord('q'):
    #     cv2.destroyAllWindows()


if __name__=="__main__":
    sift = cv2.xfeatures2d.SIFT_create()
    # sift = cv2.ORB_create()
    # sift = cv2.xfeatures2d.SURF_create()

    timg = cv2.imread('template.png')
    timggray= cv2.cvtColor(timg,cv2.COLOR_BGR2GRAY)
    # kernel = np.ones((5,5), np.uint8)
    # timggray = cv2.erode(timggray, kernel, iterations=1)
    # ret,threshimg = cv2.threshold(timggray,100,255,cv2.THRESH_BINARY)
    tkp,tdes = sift.detectAndCompute(timggray,None)

    timg2 = cv2.imread('template2.png')
    timggray2 = cv2.cvtColor(timg2,cv2.COLOR_BGR2GRAY)
    tkp2,tdes2 = sift.detectAndCompute(timggray2,None)
    timg3 = cv2.imread('template3.png')
    timggray3 = cv2.cvtColor(timg3,cv2.COLOR_BGR2GRAY)
    tkp3,tdes3 = sift.detectAndCompute(timggray3,None)

    timg = cv2.drawKeypoints(timggray,tkp,timg, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imwrite('sift_keypoints.jpg',timg)
    timg2 = cv2.drawKeypoints(timggray2,tkp2,timg2, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imwrite('sift_keypoints2.jpg',timg2)
    timg3 = cv2.drawKeypoints(timggray3,tkp3,timg3, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imwrite('sift_keypoints3.jpg',timg3)


    for i in range(1560,1575):
    # for i in range(1607,1610):

        matchimg = 'IMG_'+str(i)+'.JPG'
        result = checkAvailability(sift, tkp, tdes, matchimg)
        # result2 = checkAvailability(sift, tkp2, tdes2, matchimg)
        # result3 = checkAvailability(sift, tkp3, tdes3, matchimg)
        # print matchimg, result, result2, result3

        print matchimg,result