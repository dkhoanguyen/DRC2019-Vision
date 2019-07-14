import cv2
import numpy as np

from Edge import *
from Line import *
from object_detection import *
from vehicle_detection import *

global Paths

class Path():

    def __init__(self):

        self.no_step = 6

        self.xm_per_px = 0.002898
        self.ym_per_px = 0.004167

        self.trajectory_x = 0
        self.trajectory_y = 0

        self.max_offset_val = 185/1.5

        self.Lane_trajectory = 0

        self.final_x = 0
        self.final_y = 0
        self.final_theta = 0

        self.go_back = False

    def plan_Path(self):
        blue_trajectory = []
        yellow_trajectory = []

        self.Lane_trajectory = []

        if Edges.blue_detected == True and Edges.yellow_detected == False:
            for i in range(len(Lane.blue_confident)):

                x = Lane.blue_x[Lane.blue_confident[i]]
                y = Lane.blue_y[Lane.blue_confident[i]]
                blue_trajectory.append([x,y])

                x = Lane.blue_x[Lane.blue_confident[i]]
                y = Lane.blue_y[Lane.blue_confident[i]]
                yellow_trajectory.append([x,y])

                x = Lane.center_point_x[Lane.blue_confident[i]]
                y = Lane.center_point_y[Lane.blue_confident[i]]

                if Objects.found == 1:
                    for i in range(len(Objects.max_offset)):

                        if x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == True:
                            x = x + self.max_offset_val
                        elif x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == False:
                            x = x - self.max_offset_val
                elif Objects.found == 2:
                    self.Lane_trajectory.append([Objects.average_x,Objects.average_y])

                self.Lane_trajectory.append([x,y])

        elif Edges.yellow_detected == True and Edges.blue_detected == False:
            for i in range(len(Lane.yellow_confident)):

                x = Lane.blue_x[Lane.yellow_confident[i]]
                y = Lane.blue_y[Lane.yellow_confident[i]]
                yellow_trajectory.append([x,y])

                x = Lane.blue_x[Lane.yellow_confident[i]]
                y = Lane.blue_y[Lane.yellow_confident[i]]
                blue_trajectory.append([x,y])

                x = Lane.center_point_x[Lane.yellow_confident[i]]
                y = Lane.center_point_y[Lane.yellow_confident[i]]

                if Objects.found == 1:
                    for i in range(len(Objects.max_offset)):

                        if x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == True:
                            x = x + self.max_offset_val
                        elif x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == False:
                            x = x - self.max_offset_val
                elif Objects.found == 2:
                    self.Lane_trajectory.append([Objects.average_x,Objects.average_y])

                self.Lane_trajectory.append([x,y])

        elif Edges.yellow_detected == True and Edges.blue_detected == True:

            if Lane.yellow_confident >= Lane.blue_confident:

                for i in range(len(Lane.yellow_confident)):

                    x = Lane.blue_x[Lane.yellow_confident[i]]
                    y = Lane.blue_y[Lane.yellow_confident[i]]
                    yellow_trajectory.append([x,y])

                    x = Lane.blue_x[Lane.yellow_confident[i]]
                    y = Lane.blue_y[Lane.yellow_confident[i]]
                    blue_trajectory.append([x,y])

                    x = Lane.center_point_x[Lane.yellow_confident[i]]
                    y = Lane.center_point_y[Lane.yellow_confident[i]]

                    if Objects.found == 1:
                        for i in range(len(Objects.max_offset)):
                            if x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == True:
                                x = x + self.max_offset_val
                            elif x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == False:
                                x = x - self.max_offset_val
                    elif Objects.found == 2:
                        self.Lane_trajectory.append([Objects.average_x,Objects.average_y])

                    self.Lane_trajectory.append([x,y])
            else:
                for i in range(len(Lane.blue_confident)):

                    x = Lane.blue_x[Lane.blue_confident[i]]
                    y = Lane.blue_y[Lane.blue_confident[i]]
                    blue_trajectory.append([x,y])

                    x = Lane.blue_x[Lane.blue_confident[i]]
                    y = Lane.blue_y[Lane.blue_confident[i]]
                    yellow_trajectory.append([x,y])

                    x = Lane.center_point_x[Lane.blue_confident[i]]
                    y = Lane.center_point_y[Lane.blue_confident[i]]

                    if Objects.found == 1:

                        for i in range(len(Objects.max_offset)):
                            if x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == True:
                                x = x + self.max_offset_val
                            elif x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == False:
                                x = x - self.max_offset_val

                    elif Objects.found == 2:
                        self.Lane_trajectory.append([Objects.average_x,Objects.average_y])

                    self.Lane_trajectory.append([x,y])

        elif Edges.yellow_detected == False and Edges.blue_detected == False:
            
            for i  in range(len(Lane.center_point_x)):
                x = Lane.center_point_x[i]
                y = Lane.center_point_y[i]

                self.Lane_trajectory.append([x,y])
            if Objects.found == 1:

                for i in range(len(Objects.max_offset)):
                    if x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == True:
                        x = x + self.max_offset_val
                    elif x == Lane.center_point_x[Objects.max_offset[i]] and Objects.left_side == False:
                        x = x - self.max_offset_val

            elif Objects.found == 2:
                self.Lane_trajectory.append([Objects.average_x,Objects.average_y])

        if (len(self.Lane_trajectory) - 1) >= 1:
            distance = abs(320 - self.Lane_trajectory[0][0])

            for i in range(1,len(self.Lane_trajectory)):
                distance = distance + abs(self.Lane_trajectory[i][0] - self.Lane_trajectory[i-1][0])

            average_distance = distance/(len(self.Lane_trajectory) - 1)
        else:
            average_distance = 0
        print(average_distance)

        if average_distance > 0 and average_distance < 50 and len(Lane.yellow_confident) > 2 and len(Lane.blue_confident) > 2 and Objects.found  == 0:
            self.go_straight = True
        else:
            self.go_straight = False

        distance = 0

        for i in range(1,len(self.Lane_trajectory)):
            distance = distance + abs(self.Lane_trajectory[i][0] - self.Lane_trajectory[i-1][0])

        if len(self.Lane_trajectory)-1 is not 0:
            average_distance = distance/(len(self.Lane_trajectory) - 1)
        else:
            average_distance = 0

        if average_distance > 80:
            self.sharp_turn = True
        else:
            self.sharp_turn = False


        print(self.sharp_turn)

        if len(self.Lane_trajectory) > 0 and len(self.Lane_trajectory) < 2 and self.go_straight == False:
            print((self.Lane_trajectory))

            self.final_x = (self.Lane_trajectory[0][0] - 320) * self.xm_per_px
            self.final_y = (480 - self.Lane_trajectory[0][1]) * self.ym_per_px

            x2 = 480
            y2 = 320
            x1 = self.Lane_trajectory[0][0]
            y1 = self.Lane_trajectory[0][1]

            if (x2-x1) == 0:
                self.final_theta = pi/2
            else:

                slope = (y2-y1)/(x2-x1)
                theta = atan(slope)
                if slope > 0:
                    self.final_theta = pi - theta
                else:
                    self.final_theta = abs(theta)

        elif len(self.Lane_trajectory) >= 2 and self.go_straight == False:

            self.final_x = (self.Lane_trajectory[1][0] - 320) * self.xm_per_px
            self.final_y = (480 - self.Lane_trajectory[1][1]) * self.ym_per_px
            x2 = self.Lane_trajectory[0][0]
            y2 = self.Lane_trajectory[0][1]
            x1 = self.Lane_trajectory[1][0]
            y1 = self.Lane_trajectory[1][1]


            if (x2-x1) == 0:
                self.final_theta = pi/2
            else:
                slope = (y2-y1)/(x2-x1)
                theta = atan(slope)
                if slope > 0:
                    self.final_theta = pi - theta
                else:
                    self.final_theta = abs(theta)
        elif len(self.Lane_trajectory) == 0 and self.go_straight == False:

            self.final_x = 0.0
            self.final_y = 0.25

            self.final_theta = pi/2

        elif len(self.Lane_trajectory) == 0:
            
            self.final_x = 0.0
            self.final_y = 0.25

            self.final_theta = pi/2

            self.go_back = True

        elif self.go_straight == True:
            i = len(self.Lane_trajectory)
            self.final_x = (self.Lane_trajectory[i-2][0] - 320) * self.xm_per_px
            self.final_y = (480 - self.Lane_trajectory[i-2][1]) * self.ym_per_px - 0.33

            self.final_theta = pi/2

        if self.final_y > 1.5:
            self.final_y = 1.5


    def show_Trajectory(self,original):
        for i in range(0,len(self.Lane_trajectory)):
            cv2.circle(original,(int(self.Lane_trajectory[i][0]),int(self.Lane_trajectory[i][1])),10,(255, 255, 255),-1)

Paths = Path()
