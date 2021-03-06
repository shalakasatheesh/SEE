import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import zmq
import time
import sys
import json
from pygments import highlight, lexers, formatters

# ZeroMQ Context
context = zmq.Context()

# Define the socket using the "Context"
sock = context.socket(zmq.PUB)
sock.bind("tcp://192.168.22.101:5680")

#Camera to CV
cap = cv2.VideoCapture(0)

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((3*5,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('../../../config/calib_images/*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (5,3), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners, (11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv2.drawChessboardCorners(img, (5,3), corners2,ret)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)


while (True):
    ret, frame = cap.read()
    # operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()

    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)

    if np.all(ids != None):
        for i, corner in enumerate(corners):
            rvec, tvec, marker_corners = aruco.estimatePoseSingleMarkers(corner, 0.05, mtx, dist) #Estimate pose of each marker and return the values rvet and tvec---different from camera coefficients
            position = tvec[0][0].tolist()
            orientation = rvec[0][0].tolist()

            dictionary = {
                            'marker_id': int(ids[i][0]), 

                            "position": [position[0], 
                                        position[1], 
                                        position[2]],

                            "orientation": [orientation[0], 
                                            orientation[1], 
                                            orientation[2]]
                        }

            formatted_json = json.dumps(dictionary, indent=5)
            sock.send(formatted_json)

            aruco.drawAxis(frame, mtx, dist, rvec[0], tvec[0], 0.1) #Draw Axis
            aruco.drawDetectedMarkers(frame, corners) #Draw A square around the markers

            ###### DRAW ID #####
            cv2.putText(frame, "Id: " + str(ids), (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
