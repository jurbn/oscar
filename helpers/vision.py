import time
import logging
import math
import statistics
import traceback
import cv2 as cv
import numpy as np

import picamera
from picamera.array import PiRGBArray

params = cv.SimpleBlobDetector_Params()
params.minThreshold = 10
params.maxThreshold = 200
params.filterByArea = True
params.minArea = 20
params.maxArea = 1000000
params.filterByCircularity = True
params.minCircularity = 0.1
params.maxCircularity = 1
params.filterByColor = False
params.filterByConvexity = False
detector = cv.SimpleBlobDetector_create(params)
r11 = np.array([0, 70, 50])
r12 = np.array([10, 255, 255])
r21 = np.array([170, 70, 50])
r22 = np.array([180, 255, 255])


# REQUIRED number of correspondences (matches) found:
MIN_MATCH_COUNT=20          # initially
MIN_MATCH_OBJECTFOUND=15   # after robust check, to consider object-found

def get_blob_old(frame):
    """ Searches for a blob and returns the center """
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, r11, r12)
    mask2 = cv.inRange(hsv, r21, r22)
    mask = mask1 | mask2    # we need two masks because hsv values start and end on red :/
    res = cv.bitwise_and(frame, frame, mask=mask)   # we apply the mask...
    # detect the blobs (detector params are outside for performance)
    keypoints = detector.detect(res)
    if not keypoints:
        biggest_kp = None
    else:   # we only keep the biggest blob!
        biggest_kp = keypoints[0]   # we start looking on the first one
        for point in keypoints:     # we filter and only return the largest
            if point.size > biggest_kp.size:
                biggest_kp = point
    return biggest_kp   # return the chonky one

def get_blob(frame):
    """Returns the keypoint that's most likely to be the ball"""
    blurred = cv.GaussianBlur(frame, (11, 11), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, r11, r12)
    mask2 = cv.inRange(hsv, r21, r22)
    mask = mask1 | mask2    # we need two masks because hsv values start and end on red :/
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
    #res = cv.bitwise_and(frame, frame, mask=mask)
    keypoints = detector.detect(mask)
    if not keypoints:
        biggest_kp = None
    else:   # we only keep the biggest blob!
        biggest_kp = keypoints[0]   # we start looking on the first one
        for point in keypoints:     # we filter and only return the largest
            if point.size > biggest_kp.size:
                biggest_kp = point
    return biggest_kp   # return the chonky one


def show_cam_blobs(robot):
    """Shows a video and marks the biggest blob if any are found"""
    while (True):
        robot.reduction = 0.25
        frame = robot.takePic()
        blob = get_blob(frame=frame)
        if blob:
            logging.info('size: {}'.format(blob.size))
            im_with_keypoints = cv.drawKeypoints(frame, [blob], np.array(
                []), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            im_with_keypoints = frame
        cv.imshow('img', im_with_keypoints)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
    cv.destroyAllWindows()

def find_my_template(robot, refFilename = "res/img/R2-D2_s.png"):
    """Takes a picture and returns True if the template has been found"""
    robot.reduction = 1
    imReference = cv.imread(refFilename, cv.IMREAD_COLOR)
    frame = robot.takePic(PI=True)  # PI==True because its better for the template recognition
    cv.imwrite('res/img/last_r2d2.png', frame)
    try:
        found = match_img(imReference, frame)
    except Exception:
        logging.warning(traceback.format_exc())
        found = False
    return found

def match_img(img1_bgr, img2_bgr):
    """img1_bgr is the real image and img2_bgr the reference one"""
     
    # Feature extractor uses grayscale images
    img1 = cv.cvtColor(img1_bgr, cv.COLOR_BGR2GRAY)
    img2 = cv.cvtColor(img2_bgr, cv.COLOR_BGR2GRAY)
    
    binary_features=True
    detector = cv.BRISK_create()
        
    # find the keypoints and corresponding descriptors
    kp1, des1 = detector.detectAndCompute(img1,None)
    kp2, des2 = detector.detectAndCompute(img2,None)

    if des1 is None or des2 is None:
        return False
    if len(des1) < MIN_MATCH_COUNT or len(des2) < MIN_MATCH_COUNT:
        return False
        
    if binary_features:
        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1,des2)
        matches = sorted(matches, key = lambda x:x.distance)
        good = matches

    if len(good)>MIN_MATCH_COUNT:
        src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        H_21, mask = cv.findHomography(src_pts, dst_pts, cv.RANSAC, 3.0)
        matchesMask = mask.ravel().tolist()
        num_robust_matches = np.sum(matchesMask)
        if num_robust_matches < MIN_MATCH_OBJECTFOUND:
            found = False
            return found
        h,w = img1.shape
        pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
        dst = cv.perspectiveTransform(pts,H_21)
        img2_res = cv.polylines(img2_bgr, [np.int32(dst)], True, color=(255,255,255), thickness=3)
        found = True
    else:
        matchesMask = None
        found = False
    return found