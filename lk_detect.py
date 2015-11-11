#!/usr/bin/env python

'''
Lucas-Kanade tracker
====================

Lucas-Kanade sparse optical flow demo. Uses goodFeaturesToTrack
for track initialization and back-tracking for match verification
between frames.

Usage
-----
lk_track.py [<video_source>]


Keys
----
ESC - exit
'''

# Python 2/3 compatibility
from __future__ import print_function

import rospy
from std_msgs.msg import Int32
import numpy as np
import cv2
import video
from common import anorm2, draw_str
from time import clock
from scipy.cluster import hierarchy
from scipy import stats

lk_params = dict( winSize  = (15, 15),
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
        self.detect_interval = 1 
        self.tracks = []
        self.obstacle = []
        self.cam = video.create_capture(video_src)
        self.frame_idx = 0
        self.prevCluster = []
        self.orb = cv2.ORB(**ORB_params)
        rospy.init_node('odroid', anonymous=True)
        self.xcoordPub = rospy.Publisher('camera_x', Int32, queue_size=10)
    
    def sendCoord(self, x):
        self.xcoordPub.publish(x)

    def findBestCluster(self, clusters):
        id = 0 
        minScore = np.inf
        for i,clust in enumerate(clusters):
            if len(clust)>=8:
                score = np.linalg.norm(stats.skewtest(clust)[0])
                if score < minScore:
                    if score < 1:
                        minScore = score
                        id = i
        if minScore == np.inf:
            return self.prevCluster
        else:
            self.prevCluster = clusters[id]
            return clusters[id]

    def findObstacle(self, points):
        Z = hierarchy.linkage(points, method='ward')
        code = hierarchy.fcluster(Z, nclusters, criterion='maxclust')
        cluster = [] 
        for i in range(nclusters):
            cluster.append(points[code==i])
        bestCluster = self.findBestCluster(cluster)
        return bestCluster 

    def run(self):
        while True:
            ret, frame = self.cam.read()
            h,w,_ = frame.shape
            #h = int((1-0.65)*h)
            #frame = frame[h:,:,:]
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            vis = frame.copy()

            if len(self.tracks) > 0:
                img0, img1 = self.prev_gray, frame_gray
                p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 2)
                p0 = self.findObstacle(p0)
                if p0.size > 0:
                    self.tracks = [[(x,y)] for x,y in p0]
                    cx,cy = np.average(p0, axis=0).astype(int)
                    p0 = p0.reshape(-1,1,2)
                    p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
                    p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
                    d = abs(p0-p0r).reshape(-1, 2).max(-1)
                    good = d < 1
                    new_tracks = []
                    for tr, (x, y), good_flag in zip(self.tracks, p1.reshape(-1, 2), good):
                        if not good_flag:
                            continue
                        tr.append((x, y))
                        if len(tr) > self.track_len:
                            del tr[0]
                        new_tracks.append(tr)
                        cv2.circle(vis, (x, y), 2, (0, 255, 0), -1)
                    self.tracks = new_tracks
                cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0, 255, 0))
                cv2.circle(vis, (cx, cy), 3, (255,0,0), 2)
                try:
                    self.sendCoord(cx)
                except rospy.ROSInterruptException:
                    pass
                draw_str(vis, (20, 20), 'track count: %d' % len(self.tracks))

            if self.frame_idx % self.detect_interval == 0:
                mask = np.zeros_like(frame_gray)
                mask[:] = 255
                for x, y in [np.int32(tr[-1]) for tr in self.tracks]:
                    cv2.circle(mask, (x, y), 5, 0, -1)
                p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
                #p = self.orb.detect(frame_gray, mask=mask)
                #p = [kp.pt for kp in p]
                if p is not None:
                    p = np.float32(p).reshape(-1,2)
                    #p = self.findObstacle(p)
                    if p is not None: 
                        for x, y in p:
                            self.tracks.append([(x, y)])
                        # trackPnts = np.float32([tr[-1] for tr in self.tracks]).reshape(-1, 2)
                        # newPnts = self.findObstacle(trackPnts)
                        # print(newPnts)
                        # newTr = []
                        # for x,y in newPnts:
                            # newTr.append([(x,y)])
                        # print(len(newTr))
                        # self.obstacle = newTr

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
