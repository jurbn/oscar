import time
import cv2 as cv
import numpy as np

params = cv.SimpleBlobDetector_Params()
params.minThreshold = 10
params.maxThreshold = 200
params.filterByArea = True
params.minArea = 50
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

def get_blob(frame):
    """ Searches for a blob and returns the center """
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #avg_brightness = np.average(np.linalg.norm(hsv, axis=2))
    #print('brightness: {}'.format(avg_brightness))
    # if avg_brightness > 140:
    #    diff = avg_brightness - 140
    #    for pixel in hsv:
    #        pixel[2] = pixel[2] - diff
    #avg_brightness = np.average(np.linalg.norm(hsv, axis=2))
    mask1 = cv.inRange(hsv, r11, r12)
    mask2 = cv.inRange(hsv, r21, r22)
    mask = mask1 | mask2
    res = cv.bitwise_and(frame, frame, mask=mask)   # we apply the mask...
    # detect the blobs (detector params are outside for performance)
    keypoints = detector.detect(res)
    if not keypoints:
        biggest_kp = None
    else:   # we only keep the biggest blob!
        biggest_kp = keypoints[0]
        for point in keypoints:
            if point.size > biggest_kp.size:
                biggest_kp = point
    return biggest_kp   # return the chonky one


def show_cam_blobs(robot):
    """Shows a video and marks the biggest blob if any are found"""
    while (True):
        tIni = time.clock()
        frame = robot.takePic()[139:179, 219:239]
        blob = get_blob(frame=frame)
        print('tiempo de procesado es: {}'.format(time.clock()-tIni))
        if blob:
            print(blob.size)
            im_with_keypoints = cv.drawKeypoints(frame, [blob], np.array(
                []), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        else:
            im_with_keypoints = frame
        cv.imshow('img', im_with_keypoints)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        tEnd = time.clock()
    cv.destroyAllWindows()