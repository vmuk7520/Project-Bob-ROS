""" Module to quantitatively compare two images """

import cv2 as cv
import numpy as np
 
# read in the images
img1 = cv.imread('test.png', cv.IMREAD_GRAYSCALE)
img2 = cv.imread('test_drawing.png', cv.IMREAD_GRAYSCALE)
assert img1 is not None, "file could not be read, check with os.path.exists()"
assert img2 is not None, "file could not be read, check with os.path.exists()"

# threshold the images to only keep the important lines
ret, thresh = cv.threshold(img1, 220, 255,0)
ret, thresh2 = cv.threshold(img2, 220, 255,0)

# find the contours of the images
contours,hierarchy = cv.findContours(thresh,2,1)
cnt1 = contours[0]
contours,hierarchy = cv.findContours(thresh2,2,1)
cnt2 = contours[0]

cv.imshow("picture", thresh)
cv.imshow("drawing", thresh2)

cv.waitKey(0)
 
# calculate and print the error between the images
ret = cv.matchShapes(cnt1,cnt2,1,0.0)
print(ret)
