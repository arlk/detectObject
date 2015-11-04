__author__ = 'arun'

import cv2
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

    while True:

        ret, frame = cap.read()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cv2.imshow("grayscale", gray)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
