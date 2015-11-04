#!/usr/bin/env python

__author__ = 'arun'

import cv2
import math
import numpy as np
from optparse import OptionParser

if __name__ == "__main__":

    parser = OptionParser()
    parser.add_option("-v", "--video", action="store", type="int", dest="video", default=0,
                      help="Enter video device number -- usually available at /dev/video")
    (opts, args) = parser.parse_args()

    cap = cv2.VideoCapture(opts.video)

    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.cv.CV_CAP_PROP_FPS, 30)

    boundaries = [
    ([10, 10, 140], [150, 150, 255])
    ]

    while True:

        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	for (lower, upper) in boundaries:
	    lower = np.array(lower, dtype = "uint8")
	    upper = np.array(upper, dtype = "uint8")

	    mask = cv2.inRange(frame, lower, upper)
	    output = cv2.bitwise_and(frame, frame, mask = mask)
	    

	# cv2.imshow("images", np.hstack([frame, output]))
	cv2.imshow("image", output[:,:,2])
	ret,thresh = cv2.threshold(output[:,:,2],150,255,cv2.THRESH_BINARY)
	contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
	for cnt in contours:
	    area = cv2.contourArea(cnt)
	    if area > 600:
		M = cv2.moments(cnt)
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		center = (cx,cy)
		radius = int(math.sqrt(area/math.pi))
		cv2.circle(frame,center,radius,(0,255,0),2)        
	# cv2.imshow("grayscale", frame)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
