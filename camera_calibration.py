import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
import os.path as path
import pickle
import glob

global get_cam_cali

def calibrate_camera(calibration_dir = 'camera_cal/', print_image = False, number_column = 9, number_row = 7):

    """
    This function takes in the images of a chessboard to calculate and output the distortion coefficients.
    The coefficients later are fed to the undistort function to calibrate the camera. 
    This process should be implemented first before loading the other parts of the project and must be implemented for every camera.

    Input: Chessboard images in different angles and direction.
    Output: a turple of calibration coefficient
    """

    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 24, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*8,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('camera_cal/*.png')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (8,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # # Draw and display the corners
            # img = cv2.drawChessboardCorners(img, (8,6), corners2,ret)
            # cv2.imshow('img',img)
            # cv2.waitKey(500)

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    calibration_coefficient = [mtx, dist, rvecs, tvecs]

    #save the distortion coefficient for future uses (save as a dictionary)
    calibration = {}
    calibration["mtx"] = mtx
    calibration["dist"] = dist
    calibration["rvecs"] = rvecs 
    calibration["tvecs"] = tvecs
    pickle.dump(calibration, open( "camera_cal/calibration_data.pickle", "wb" ) )

    return calibration_coefficient

def get_calibration_coefficient():
    
    calibration_cache = 'camera_cal/calibration.p'
    get_calibration = open("camera_cal/calibration_data.pickle","rb")
    calibration = pickle.load(get_calibration)

    mtx = calibration["mtx"]
    dist = calibration["dist"]
    rvecs = calibration["rvecs"]
    tvecs = calibration["tvecs"]

    calibration_coefficient = [mtx, dist, rvecs, tvecs]

    get_cam_cali = False

    return calibration_coefficient

def camera_undistortion(img,calibration_coefficient):
    return cv2.undistort(img,calibration_coefficient[0],calibration_coefficient[1],None,calibration_coefficient[0])


# calibration_coefficient = get_calibration_coefficient()
# print(calibration_coefficient[0])