#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
import numpy as np
import math
import cv2
import video
from common import anorm2, draw_str
from time import clock
from scipy.cluster import hierarchy
from scipy import stats
from scipy.spatial import distance

lk_params = dict( winSize  = (20, 20),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )
nclusters = 5 

ORB_params = dict( nfeatures = 500,
                       scaleFactor = 1.1)


class App:
    def __init__(self, video_src):
        self.track_len = 10
        self.detect_interval = 5 
        self.h = 0
        self.tracks = []
        self.obstacle = []
        self.cam = video.create_capture(video_src)
        self.cam.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 854)
        self.cam.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)
        self.cam.set(cv2.cv.CV_CAP_PROP_FPS, 30)
        self.frame_idx = 0
        self.notDetected = 0
        self.globalMinDist = 300 
        self.prevCluster = []
        self.meanDist = 0
        self.rhoDot = 0
        #self.orb = cv2.ORB(**ORB_params)
        self.fast = cv2.FastFeatureDetector()
        rospy.init_node('odroid', anonymous=True)
        self.xcoordPub = rospy.Publisher('camera_x', Int32, queue_size=10)
        self.rhodotPub = rospy.Publisher('rho_dot', Float64, queue_size=10)
    
    def sendCoord(self, x, y):
        x -= 640
	self.xcoordPub.publish(x)
        self.rhodotPub.publish(y)

    def findBestCluster(self, clusters):
        id = 0 
        minScore = np.inf 
        DONE = False
        minDistance = self.globalMinDist
        mindiffScore = 4444 
        prevAvg = np.average(self.prevCluster, axis=0) 
        #print("Nclusters",nclusters)
        for i,clust in enumerate(clusters):
            #print len(clust)
            if len(clust)>=8:
                score = np.linalg.norm(stats.skewtest(clust)[0])
                var = np.var(clust, axis=0)
                avg = np.average(clust, axis=0)
                dist = np.linalg.norm(prevAvg - avg)
                if np.isnan(dist):
                    dist = 0 
                diffScore = math.fabs(var[0] - var[1])
                #print("hello")
                if score < minScore:
                    #print(score, diffScore, avg[1])
                    if diffScore<mindiffScore and avg[1]>self.h/2 and dist<minDistance:
                        DONE = True
                        minScore = score
                        mindiffScore = diffScore
                        minDistance = dist
                        id = i

        if DONE == False:
            #print("NOT DETECTED")
            self.globalMinDist += 10
            self.notDetected += 1
            #return self.prevCluster
            return []
        else:
            self.globalMinDist -= 50
            if self.globalMinDist < 25:
                self.globalMinDist =25 
            self.notDetected = 0
            #print("FINAL:", mindiffScore, minScore, minDistance)
            self.prevCluster = clusters[id]
            return clusters[id]

    def findObstacle(self, points):
        global nclusters
        Z = hierarchy.linkage(points, method='ward')
        if len(Z) > 0:
            code = hierarchy.fcluster(Z, 1000, criterion = 'distance')#nclusters, criterion='maxclust')
            nclusters = np.max(code)
            cluster = [] 
            for i in range(1,nclusters+1):
                cluster.append(points[code==i])
            bestCluster = self.findBestCluster(cluster)
            prevDist = self.meanDist
            #if len(bestCluster) > 0:
                #distn = distance.pdist(bestCluster)
                #self.meanDist = np.average(distn)
            #print self.meanDist, self.rhoDot
            self.rhoDot = (self.meanDist - prevDist)*30
        else:
            bestCluster = []
        return bestCluster 

    def run(self):
        while True:
            ret, frame = self.cam.read()
            self.h,w,_ = frame.shape
            #h = int((1-0.65)*h)
            #frame = frame[h:,:,:]
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()

            if len(self.tracks) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 2)
                #p0 = self.findObstacle(p0)
                if len(p0) > 0:
                    self.tracks = [[(x,y)] for x,y in p0]
                    p0 = p0.reshape(-1,1,2)
                    p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
                    p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
                    d = abs(p0-p0r).reshape(-1, 2).max(-1)
                    good = d < 1
                    new_tracks = []
                    for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                        if not good_flag:
                            #print ("OH NO!")
                            continue
                        tr.append((x, y))
                        if len(tr) > self.track_len:
                            del tr[0]
                        new_tracks.append(tr)
                        cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
                    self.tracks = new_tracks
                    p3 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 2)
                    if len(p3) > 0:
                        p3 = self.findObstacle(p3)
                        if len(p3)>0:
                            cx,cy = np.average(p3, axis=0).astype(int)
                            for x, y in p3:                 
                                cv2.circle(vis, (x, y), 2, (255, 255, 0), -1)
                            cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                            cv2.circle(vis, (cx, cy), 4, (0,0,255), 2)
                            try:
                                self.sendCoord(cx, self.rhoDot)
                            except rospy.ROSInterruptException:
                                pass
                    draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

            if self.notDetected > 0:
                try:
                    self.sendCoord(9999+640, self.rhoDot)
                except rospy.ROSInterruptException:
                    pass

                
            if self.frame_idx % self.detect_interval == 0:
                mask = np.zeros_like(frame_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                    cv2.circle(mask, (x, y), 5, 0, -1)
                p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                if p is not None:
                    p = np.float32(p).reshape(-1,2)
                    if p is not None:
                        for x, y in p:
                            self.tracks.append([(x, y)])

            self.frame_idx += 1
            self.prev_gray = frame_gray
            cv2.imshow('lk_track', vis)

            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                break

def main():
    import sys
    try:
        video_src = sys.argv[1]
    except:
        video_src = 0

    print(__doc__)
    App(video_src).run()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
