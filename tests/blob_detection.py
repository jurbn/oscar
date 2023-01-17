import cv2 as cv
import numpy as np
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

def detect_blob():
    print('a')
    frame = cv.imread('res/img/frame.png', cv.IMREAD_COLOR)
    blurred = cv.GaussianBlur(frame, (11, 11), 5)
    cv.imwrite('res/img/Blurred_Image.png', blurred)

    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(hsv, r11, r12)
    mask2 = cv.inRange(hsv, r21, r22)
    mask = mask1 | mask2    # we need two masks because hsv values start and end on red :/
    color_masked = cv.bitwise_and(frame, frame, mask=mask)
    cv.imwrite('res/img/Color_Masked_Image.png', color_masked)

    mask = cv.erode(mask, None, iterations=2)
    eroded = cv.bitwise_and(frame, frame, mask=mask)
    cv.imwrite('res/img/Eroded_Image.png', eroded)

    mask = cv.dilate(mask, None, iterations=1)
    res = cv.bitwise_and(frame, frame, mask=mask)
    dilated_bgr = cv.cvtColor(res, cv.COLOR_HSV2BGR)
    cv.imwrite('res/img/Dilated_Image.png', res)

    keypoints = detector.detect(mask)
    if not keypoints:
        biggest_kp = None
    else:   # we only keep the biggest blob!
        biggest_kp = keypoints[0]   # we start looking on the first one
        for point in keypoints:     # we filter and only return the largest
            if point.size > biggest_kp.size:
                biggest_kp = point
    if biggest_kp:
        im_with_keypoints = cv.drawKeypoints(frame, [biggest_kp], np.array([]), (255, 255, 255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv.imwrite('res/img/Blob.png', im_with_keypoints)
    else: print(':(')

if __name__ == '__main__':
    detect_blob()