import numpy as np
import cv2
import cv2.aruco as aruco

'''
    drawMarker(...)
        drawMarker(dictionary, id, sidePixels[, img[, borderBits]]) -> img
'''

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

# second parameter is id number
# last parameter is total image size
img = aruco.drawMarker(aruco_dict, 0, 200)
cv2.imwrite("test_marker_0.jpg", img)

cv2.imshow('frame', img)
cv2.waitKey(0)
cv2.destroyAllWindows()