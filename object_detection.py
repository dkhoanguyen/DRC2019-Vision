import numpy
import cv2

from Edge import *
from Line import *
from perspective_region import *
from Utils import *

global Objects

class Object():

    def __init__(self):

        self.center = [0,0]
        self.area = 0
        self.found = 0

        self.left_side = True
        self.right_side = True
        self.distance = 0

        self.angle = 0


    def getObject(self,img,original):
        self.found = 0
        self.center = []
        self.distance = 0
        self.area = []

        img = region_of_interest(img)

        img2 = img.astype(np.uint8)

        contours_no = []

        i, contours,h = cv2.findContours(img2,1,2)

        if len(contours) != 0:

            for i in range(len(contours)):
                area = cv2.contourArea(contours[i])

                if area > 500:
                    self.found = self.found + 1
                    contours_no.append(i)

            if self.found == 1 :
                M = cv2.moments(contours[contours_no[0]])
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                cv2.circle(original,(cx,cy),10,(255, 105, 180),-1)

                self.center = [cx,cy]

            elif self.found == 2:
                for i in range(len(contours_no)):
                    M = cv2.moments(contours[contours_no[i]])
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    cv2.circle(original,(cx,cy),10,(255, 105, 180),-1)

                    (self.center).append([cx,cy])


    def get_ObjectPoints(self,img,original,no_step):

        step = np.int(img.shape[0]//no_step)

        self.getObject(img,original)

        if self.found == 1:
            self.boundary_x = [self.center[0] - 185/2,self.center[0] + 185/2]
            self.boundary_y = [self.center[1] + 124/2,self.center[1] - 124/2]

            if self.center[1] % step == 0:
                position = no_step - 1 - (self.center[1] / step)
                self.max_offset = [position - 2,position - 1,position,position + 1]
                self.ave_offset = [position - 2, position + 2]

            else:
                position = no_step - 1 - int(self.center[1] / step)
                if float((self.center[1] % step) / step) < 0.65:
                    self.max_offset = [position - 2,position - 1,position, position + 1]
                    self.ave_offset = [position - 2, position + 2]
                else:
                    self.max_offset = [position - 1,position, position + 1, position + 2]
                    self.ave_offset = [position - 1, position + 3]

            if  self.center[0] - Lane.blue_x[position] < (Lane.yellow_x[position] - self.center[0]):
                self.left_side = True
                self.right_side = False
            else:
                self.left_side = False
                self.right_side = True

            if Edges.yellow_detected == False and Edges.blue_detected == False:
                if self.center[0] < img.shape[1]//2:
                    self.left_side = True
                    self.right_side = False
                else:
                    self.left_side = False
                    self.right_side = True

        elif self.found == 2:
            if self.center[0][0] > self.center[1][0]:
                self.average_x = self.center[1][0] + 0.5*(self.center[0][0] - self.center[1][0])
            else:
                self.average_x = self.center[0][0] + 0.5*(self.center[1][0] - self.center[0][0])

            if self.center[0][1] > self.center[1][1]:
                self.average_y = self.center[1][1] + 0.5*(self.center[0][1] - self.center[1][1])
            else:
                self.average_y = self.center[0][1] + 0.5*(self.center[1][1] - self.center[0][1])

            cv2.circle(original,(int(self.average_x),int(self.average_y)),10,(255, 180, 180),-1)

    def getFound(self):
        return self.found

    def visualizeObjects(self,original):
        cv2.circle(original,(int(self.boundary_x[0]),int(self.boundary_y[0])),10,(255, 105, 180),-1)
        cv2.circle(original,(int(self.boundary_x[0]),int(self.boundary_y[1])),10,(255, 105, 180),-1)
        cv2.circle(original,(int(self.boundary_x[1]),int(self.boundary_y[1])),10,(255, 105, 180),-1)
        cv2.circle(original,(int(self.boundary_x[1]),int(self.boundary_y[0])),10,(255, 105, 180),-1)

Objects = Object()
