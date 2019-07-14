#!/usr/bin/env python3

import numpy as np
import cv2

from Edge import *
from perspective_region import *
from Utils import *

from Line import *
from object_detection import *
# from vehicle_detection import *
from camera_calibration import *

calibration_coefficient = calibrate_camera()
cap = cv2.VideoCapture('run3.avi')

while(cap.isOpened()):
    ret, frame = cap.read()

    frame = camera_undistortion(frame,calibration_coefficient)

    Edges._Canny(frame)

    print(hello)

    # Edges._object_thresholding(frame)

    # warped = perspective_transform(frame)

    # Objects.get_ObjectAngle(Edges.Objects)

    # print(Objects.angle)

    Edges._get_blue_yellow_Lines(frame)

    # warped_blue = perspective_transform(Edges.blue_Lines)

    # warped_yellow = perspective_transform(Edges.yellow_Lines)

    Lane._sliding_window_search(Edges.Lines)

    Lane._fit_polynomial(Edges.Lines)
    
    Lane.get_LaneAngle(Edges.warped)

    Lane_angle = Lane.angle

    Lane.get_LaneOffset(Edges.Lines)

    Lane_offset = Lane.angle

    print("Lane Angle: " + str(Lane_angle) + " - Lane Offset: " + str(Lane_offset))

    cv2.imshow("img",Edges.warped)

    Lane.first_frame = False

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
