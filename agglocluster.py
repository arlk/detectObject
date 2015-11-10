#!/usr/bin/env python

__author__ = 'arun'

import cv2
import math
import numpy as np
from optparse import OptionParser
from scipy.cluster import vq
from scipy.cluster import hierarchy
from scipy.spatial import distance
from scipy import stats

def bestCluster(cluster, distCluster):
    var = np.inf
    id = 0
    for i,group in enumerate(distCluster):
        if group.shape[0]>clusterThresh:
            if np.var(group) < var:
                var = np.var(group)
                id = i
    #print var,stats.normaltest([distCluster[id]],None)
    return var,id

def saturate(num,min,max):
    if num<min:
        num=min
    elif num>max:
        num=max
    return num

def checkpoints(pnts):
    global nclusters
    distMat = distance.pdist(pnts)
    newpnts = np.empty([1,2])
    newpnts[0] = pnts[-1]
    b = 0
    for i in range(pnts.shape[0]-1):
        a = b
        b += (pnts.shape[0]-1) - i 
        if np.all(distMat[a:b]>distThresh):
            newpnts = np.append(newpnts, [pnts[i]], axis=0)


    nclusters = saturate(nclusters,minClusters,maxClusters)
    if newpnts.shape[0] < nclusters:
        for i in range(nclusters-newpnts.shape[0]):
            while True:
                randPnt = [(h-1)*np.random.randn(2)]
                cdistMat = distance.cdist(randPnt,newpnts)
                if np.all(cdistMat>distThresh):
                    newpnts = np.append(newpnts, randPnt, axis=0)
                    break

    return newpnts

if __name__ == "__main__":

    parser = OptionParser()
    parser.add_option("-v", "--video", action="store", type="int", dest="video", default=0,
                      help="Enter video device number -- usually available at /dev/video")
    (opts, args) = parser.parse_args()

    cap = cv2.VideoCapture(opts.video)

    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 854)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.cv.CV_CAP_PROP_FPS, 30)

    boundaries = [
    ([10, 10, 140], [150, 150, 255])
    ]
    
    nfeatures = 500
    distThresh = 200
    minClusters = 10 
    maxClusters = 10 
    scaleFactor = 1.01
    nclusters = minClusters 
    oldCenteroids = None
    while True:

        ret, frame = cap.read()
        h,w,_ = frame.shape
        # h=int((1-0.65)*h)
        #frame = frame[h:,:,:]
	frame = cv2.bilateralFilter(frame,5,75,75)
        
        
        orb = cv2.ORB(int(nfeatures),scaleFactor)

        kp = orb.detect(frame, None)

        kp, des = orb.compute(frame, kp)

        
        kpoints = np.asarray([point.pt for point in kp])

        
        clusterThresh = 0.2*kpoints.shape[0]


        Z = hierarchy.linkage(kpoints)
        code = hierarchy.fcluster(Z, nclusters, criterion='maxclust')

#         try:
            # if oldCenteroids==None:
                # centers, distort = vq.kmeans(kpoints, nclusters, iter=10)
            # else:
                # centers, distort = vq.kmeans(kpoints, oldCenteroids)
        # except:
            # centers = oldcenters
            # kpoints = oldkpoints
            # pass
        # code, dist = vq.vq(kpoints, centers)
        # oldcenters = centers
        # oldkpoints = kpoints
    
        
        cluster = [] 
        # distCluster = []
        for i in range(nclusters):
            cluster.append(kpoints[code==i])   
            # distCluster.append(dist[code==i])
        
        # var, id = bestCluster(cluster)
        # if var == np.inf:
            # nclusters -= 1
        # print centers.shape

        # newFrame = frame.copy()
        newFrame = cv2.drawKeypoints(frame, kp, color=(0,255,0), flags=0)
        
        colors = [(255,0,255),(255,255,0),(0,255,255),(100,150,200),(200,150,100),(50,250,50),(100,0,255),(155,0,60)]
        for id,color in enumerate(colors[:nclusters]):


            clusterPoints = tuple(map(tuple, cluster[id].astype(int)))
        
        # cnt = np.array([map(tuple, cluster[id].astype(int))]) 
        # hull = cv2.convexHull(cnt)
        # M = cv2.moments(hull)
        # if M['m00'] != 0:
            # cx = int(M['m10']/M['m00'])
            # cy = int(M['m01']/M['m00'])
            # if var < 1000:
                # cv2.drawContours(newFrame, [hull], 0, (255, 0 , 255), 2)
                # cv2.circle(newFrame, (cx, cy), 3, (255,0,0), 2)
            # if distance.euclidean(np.array([cx,cy]), centers[id]) > 0.1*distThresh:
                # nclusters += 1
            for i,points in enumerate(clusterPoints):
                cv2.circle(newFrame, points, 1, color, 2)

        # oldCenteroids = centers
        # oldCenteroids = checkpoints(oldCenteroids)

        # centers = tuple(map(tuple, centers.astype(int)))
        # for i,center in enumerate(centers):
            # cv2.circle(newFrame, center, 2, (0,0,255), 2)    

        cv2.imshow("grayscale", newFrame)


        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
