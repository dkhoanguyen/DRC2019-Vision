import cv2
import numpy as np
import perspective_region as warp
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from math import *
from Edge import *

global Lane

class Line():

    def __init__(self):

        self.ym_per_pix = 0.0041667
        self.xm_per_pix = 0.002898
        self.first_frame = True

        self.lane_detected = False
        self.left_lane = False
        self.right_lane = False

        self.yellow_x = 0
        self.yellow_y = 0

        self.blue_x = 0
        self.blue_y = 0

        self.current_x = 0
        self.current_y = 0
        self.distance = 0

        self.current_fit = 0
        self.fit = np.zeros(3)
        self.best_fit = 0

        self.current_fitx = 0
        self.fitx = None
        self.best_fitx = np.zeros(360)
        self.ploty = 0

        self.lane_pose = 0
        self.center_dist = 0 
        self.n_frame = 10
        self.result = 0

        self.current_angle = 0
        self.angle = 0
        self.current_slope = 0
        self.slope = 0
        self.average_slope = 0
        
        self.offset = 0
        self.average_offset = 0
        self.current_offset = 0

        self.offset_threshold = 25

    def getBlue_Line(self,img, no_step = 20):

        self.blue_confident = []

        margin = 250
        
        margin = 250
        step = np.int(img.shape[0]//no_step)

        bottom_half = img[int(img.shape[0]*0.5):int(img.shape[0]*1),:]
        histogram = np.sum(bottom_half,axis = 0)

        x_ind = []
        y_ind = []

        start_x = np.argmax(histogram) 
        start_y = img.shape[0] - 1

        if Edges.blue_detected == False:
            start_x = -5

        current_x = start_x
        current_y = start_y

        x_ind.append(current_x)
        y_ind.append(current_y)

        nonzero = np.nonzero(img)
        nonzerox = np.array(nonzero[1])
        nonzeroy = np.array(nonzero[0]) 

        left_x = current_x - margin
        right_x = current_x + margin
        current_y = img.shape[0] - 1

        current_nonzerox_ind = ((nonzerox >= left_x) &  (nonzerox < right_x) & (nonzeroy == current_y)).nonzero()[0]

        if len(current_nonzerox_ind) > 5:
            self.blue_confident.append(0)

        for y in range(no_step):
            left_x = current_x - margin
            right_x = current_x + margin
            current_y = img.shape[0] - (y + 1)*step

            current_nonzerox_ind = ((nonzerox >= left_x) &  (nonzerox < right_x) & (nonzeroy == current_y)).nonzero()[0]

            if len(current_nonzerox_ind) > 5:
                current_x = np.int(np.mean(nonzerox[current_nonzerox_ind]))
                self.blue_confident.append(y+1)

            elif start_x != -5 :
                current_x = current_x + (x_ind[y] - x_ind[y -1])

            x_ind.append(current_x)
            y_ind.append(current_y)

        return x_ind,y_ind

    def getYellow_Line(self,img, no_step = 20):

        self.yellow_confident = []

        margin = 250
        step = np.int(img.shape[0]//no_step)

        bottom_half = img[int(img.shape[0]*0.5):int(img.shape[0]*1),:]
        histogram = np.sum(bottom_half,axis = 0)

        x_ind = []
        y_ind = []

        start_x = np.argmax(histogram) 
        start_y = img.shape[0] - 1

        if Edges.yellow_detected == False:
            start_x = img.shape[1] + 5

        current_x = start_x
        current_y = start_y

        x_ind.append(current_x)
        y_ind.append(current_y)

        nonzero = np.nonzero(img)

        nonzerox = np.array(nonzero[1])
        nonzeroy = np.array(nonzero[0]) 

        left_x = current_x - margin
        right_x = current_x + margin
        current_y = img.shape[0] - 1

        current_nonzerox_ind = ((nonzerox >= left_x) &  (nonzerox < right_x) & (nonzeroy == current_y)).nonzero()[0]

        if len(current_nonzerox_ind) > 5:
            self.yellow_confident.append(0)

        for y in range(no_step):
            left_x = current_x - margin
            right_x = current_x + margin
            current_y = img.shape[0] - (y + 1)*step

            current_nonzerox_ind = ((nonzerox >= left_x) &  (nonzerox < right_x) & (nonzeroy == current_y)).nonzero()[0]
            
            if len(current_nonzerox_ind) > 5 and Edges.yellow_detected == True:
                current_x = np.int(np.mean(nonzerox[current_nonzerox_ind]))
                self.yellow_confident.append(y+1)

            elif start_x != img.shape[1] + 5:
                current_x = current_x + (x_ind[y] - x_ind[y -1])
            else: 
                current_x = current_x
                
            x_ind.append(current_x)
            y_ind.append(current_y)

        return x_ind,y_ind

    def get_YellowCoordinate(self,img,original):
        self.yellow_x, self.yellow_y = self.getYellow_Line(img,6)

    def get_BlueCoordinate(self,img,original):
        self.blue_x, self.blue_y = self.getBlue_Line(img,6)
    
    def get_Center(self,original):
        self.center_point_x = []
        self.center_point_y = self.yellow_y
        eliminate = 0

        if Edges.yellow_detected == True and Edges.blue_detected == True:
            for i in range(len(self.yellow_x)):
                if self.blue_x[i] > self.yellow_x[i]:
                    eliminate = eliminate + 1

            if eliminate > 2:
                for i in range(len(self.yellow_x)):
                    self.blue_x[i] = self.yellow_x[i] - (self.yellow_x[0])
                    self.blue_confident = []

        elif Edges.yellow_detected == False and Edges.blue_detected == True:
            for i in range(len(self.yellow_x)):
                self.yellow_x[i] = self.blue_x[i] + (self.yellow_x[0] - self.blue_x[0])

        for i in range(len(self.yellow_x)):
            self.center_point_x.append(self.blue_x[i] + int(0.5*(self.yellow_x[i] - self.blue_x[i])))

        self.center_point_y = self.yellow_y
        
    def pointVisulization(self,original):
        for i in range(0,len(self.center_point_x)):
            cv2.circle(original,(self.center_point_x[i],self.center_point_y[i]),10,(0, 0, 255),-1)
            cv2.circle(original,(self.blue_x[i],self.blue_y[i]),10,(255, 0, 0),-1)
            cv2.circle(original,(self.yellow_x[i],self.yellow_y[i]),10,(0, 255, 0),-1)



Lane = Line()