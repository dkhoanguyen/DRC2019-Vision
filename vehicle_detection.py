import numpy as np
import cv2

from Edge import *

global Vehicles

class Vehicle():

    def __init__(self):
        self.position = []
        self.found = 0

    def trackVehicle(self,img):
        self.found = 0

        img = region_of_interest(img)

        img2 = img.astype(np.uint8)

        contours_no = []

        i, contours,h = cv2.findContours(img2,1,2)

        if len(contours) != 0:

            for i in range(len(contours)):
                area = cv2.contourArea(contours[i])

                if area > 2000:
                    self.found = 1
                    contours_no = i

            if self.found == 1 :
                M = cv2.moments(contours[contours_no])
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # cv2.circle(original,(cx,cy),10,(255, 50, 50),-1)

                self.position = [cx,cy]
            else:
                self.position = []

    def stopNow(self):
        if self.found is not 0:
            self.stop = True
        else:
            self.stop = False

    def getFound(self):
        return self.found

    def avoid(self,img):
        self.trackVehicle(img)
        if self.found is not 0:
            do = 1
        else:
            do = 0

Vehicles = Vehicle()
