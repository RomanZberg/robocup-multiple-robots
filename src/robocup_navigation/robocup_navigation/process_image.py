#!/usr/bin/env python3

import cv2
import numpy as np
import math

MARKER_LENGTH = 0.13
camMat = np.matrix([[381.36246688113556, 0., 320.5],[0., 381.36246688113556, 240.5],[0., 0., 1.]])
distCoef = np.array([0., 0., 0., 0., 0. ])

def find_marker(image):
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image,arucoDict,parameters=arucoParams)
    if np.all(ids is not None):
        print(ids)
        #image_marker = draw_marker(image, corners, ids)
        #image_axis = marker_pos(image_marker,corners)
        return corners
    else:
        return 0

def draw_marker(image, corners, ids):
    colour = (0,255,0)
    return cv2.aruco.drawDetectedMarkers(image, corners, ids, colour)

def grayscale_image(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

def marker_pos(corners):
    rvecs, tvecs, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camMat, distCoef)
    #image_axis = cv2.drawFrameAxes(image, camMat, distCoef, rvecs, tvecs, 0.1, 3)
    return tvecs , rvecs

def rodrigues(rvecs):
    return cv2.Rodrigues(rvecs)

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

def wait_on_gui():
    cv2.waitKey(2)