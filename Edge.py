import numpy as np
import cv2
from perspective_region import *

global Edges

class Edge():

    def __init__(self):

        self.Canny = 0

        self.current_color_mask = 0
        self.current_abs_mask = 0
        self.current_mag_mask = 0
        self.current_dir_mask = 0
        self.current_magdir_mask = 0
        self.combined_masked = 0
        self.warped = 0
        self.filtered = 0

        self.sobel_kernel = 11
        self.abs_sobel_kernel = 15
        self.mag_sobel_kernel = 11
        self.dir_sobel_kernel = 11

        self.lower_color_threshold = 180
        self.upper_color_threshold = 255

        self.lower_absolute_threshold = 220
        self.upper_absolute_threshold = 255

        self.lower_mag_threshold = 20
        self.upper_mag_threshold = 35

        self.lower_dir_threshold = 0.1
        self.upper_dir_threshold = 1.4

        self.blue_Lines = 0
        self.yellow_Lines = 0
        self.Lines = 0

        self.Objects = 0

        self.Vehicle = 0


    def _change_color_threshold(self,threshold = (0,255)):
        self.lower_color_threshold = threshold[0]
        self.upper_color_threshold = threshold[1]


    def _color_threshold(self,img,color_space = 'RGB',channel = 'R'):
        if color_space == 'RGB':
            rgb = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
            if channel == 'R':
                new_img = rgb[:,:,0]

            elif channel == 'G':
                new_img = rgb[:,:,1]

            elif channel == 'B':
                new_img = rgb[:,:,2]

        elif color_space == 'HSV':
            hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
            if channel == 'H':
                new_img = hsv[:,:,0]

            elif channel == 'S':
                new_img = hsv[:,:,1]

            elif channel == 'V':
                new_img = hsv[:,:,2]

        elif color_space == 'LAB':
            lab = cv2.cvtColor(img,cv2.COLOR_BGR2Lab)
            if channel == 'L':
                new_img = lab[:,:,0]

            elif channel == 'A':
                new_img = lab[:,:,1]

            elif channel == 'B':
                new_img = lab[:,:,2]

        elif color_space == 'HLS':
            hls = cv2.cvtColor(img,cv2.COLOR_BGR2HLS)
            if channel == 'H':
                new_img = hls[:,:,0]

            elif channel == 'L':
                new_img = hls[:,:,1]

            elif channel == 'S':
                new_img = hls[:,:,2]

        new_img = new_img*(255/np.max(new_img))
        binary_output = np.zeros_like(new_img)
        binary_output[(new_img >= self.lower_color_threshold) & (new_img <= self.upper_color_threshold)] = 1

        self.current_color_mask = binary_output

    # def _sobel_absolute(self,img,orient = 'x'):
    #     # Grayscale
    #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #     if orient == 'x':

    #         sobel = cv2.Sobel(gray,cv2.CV_64F,1,0,ksize=self.sobel_kernel)
    #         abs_sobel = np.absolute(sobel)

    #     else:

    #         sobel = cv2.Sobel(gray,cv2.CV_64F,0,1,ksize=self.sobel_kernel)
    #         abs_sobel = np.absolute(sobel)

    #     # Scale the result to an 8-bit range (0-255)
    #     scaled_sobel = np.uint8(255*(abs_sobel/np.max(abs_sobel)))
    #     # Apply lower and upper thresholds
    #     binary_output = np.zeros_like(scaled_sobel)
    #     # Create binary_output
    #     binary_output[(scaled_sobel >= self.lower_absolute_threshold) & (scaled_sobel <= self.upper_absolute_threshold)] = 1
    #     self.current_abs_mask = binary_output

    # def _sobel_magnitute(self,img):

    #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #     sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=self.mag_sobel_kernel)
    #     sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=self.mag_sobel_kernel)

    #     sobel_mag = np.sqrt(np.square(sobel_x) + np.square(sobel_y))

    #     # Rescale to 8 bit
    #     scale_factor = np.max(sobel_mag)/255
    #     sobel_mag = (sobel_mag/scale_factor).astype(np.uint8)

    #     # Create a binary image of ones where threshold is met, zeros otherwise
    #     binary_output = np.zeros_like(sobel_mag)
    #     binary_output[(sobel_mag >= self.lower_mag_threshold) & (sobel_mag <= self.upper_mag_threshold)] = 255

    #     self.current_mag_mask = binary_output

    # def _sobel_direction(self,img):
    #     # Grayscale
    #     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    #     # Calculate the x and y gradients
    #     sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=self.dir_sobel_kernel)
    #     sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=self.dir_sobel_kernel)

    #     # Take the absolute value of the gradient direction,
    #     absgraddir = np.arctan2(np.absolute(sobely), np.absolute(sobelx))

    #     # apply a threshold, and create a binary image result
    #     binary_output =  np.zeros_like(absgraddir)
    #     binary_output[(absgraddir >= self.lower_dir_threshold) & (absgraddir <= self.upper_dir_threshold)] = 255

    def _Canny(self,img):
        self.Canny = cv2.Canny(img,150,200)


    def _object_thresholding(self,img):

        # # Lab Color Thresholding Blue
        # self._change_color_threshold((0,50))
        # self._color_threshold(img,color_space = 'LAB',channel = 'L')
        # blue_Lab_L = self.current_color_mask
        #
        # self._change_color_threshold((0,100))
        # self._color_threshold(img,color_space = 'LAB',channel = 'A')
        # blue_Lab_a = self.current_color_mask
        #
        # self._change_color_threshold((0,110))
        # self._color_threshold(img,color_space = 'LAB',channel = 'B')
        # blue_Lab_b = self.current_color_mask
        #
        # blue_Lab = np.zeros_like(blue_Lab_b)
        # blue_Lab[((blue_Lab_L == 1) & (blue_Lab_a == 1) & (blue_Lab_b == 1))] = 1
        #
        # # HSV Color Thresholding Blue
        # self._change_color_threshold((80,150))
        # self._color_threshold(img,color_space = 'HSV',channel = 'H')
        # blue_hsv_h = self.current_color_mask
        #
        # self._change_color_threshold((0,150))
        # self._color_threshold(img,color_space = 'HSV',channel = 'S')
        # blue_hsv_s = self.current_color_mask
        #
        # self._change_color_threshold((0,129))
        # self._color_threshold(img,color_space = 'HSV',channel = 'V')
        # blue_hsv_v = self.current_color_mask
        #
        # blue_hsv = np.zeros_like(blue_hsv_h)
        # blue_hsv[((blue_hsv_h == 1) & (blue_hsv_s == 1) & (blue_hsv_v == 1))] = 1
        # self.current_color_mask = blue_hsv

        # HSV Color Thresholding
        self._change_color_threshold((125,150))
        self._color_threshold(img,color_space = 'HSV',channel = 'H')
        object_hsv_h = self.current_color_mask

        self._change_color_threshold((0,125))
        self._color_threshold(img,color_space = 'HSV',channel = 'S')
        object_hsv_s = self.current_color_mask

        self._change_color_threshold((0,100))
        self._color_threshold(img,color_space = 'HSV',channel = 'V')
        object_hsv_v = self.current_color_mask

        object_hsv = np.zeros_like(object_hsv_v)
        object_hsv[((object_hsv_h == 1) & (object_hsv_s == 1) & (object_hsv_v == 1))] = 255
        object_hsv = region_of_interest(object_hsv)
        object_hsv = perspective_transform(object_hsv)

        self.contours_Filtering(object_hsv,250)

        self.Objects = self.filtered
        kernel = np.ones((11,11), np.uint8)
        self.Objects = cv2.dilate(self.Objects, kernel, iterations = 1)

    def vehicle_Thresholding(self,img):
        self._change_color_threshold((147,255))
        self._color_threshold(img,color_space='RGB',channel='R')
        vehicle_rgb_r = self.current_color_mask

        self._change_color_threshold((0,100))
        self._color_threshold(img,color_space='RGB',channel='G')
        vehicle_rgb_g = self.current_color_mask

        self._change_color_threshold((0,95))
        self._color_threshold(img,color_space='RGB',channel='B')
        vehicle_rgb_b = self.current_color_mask

        vehicle_rgb = np.zeros_like(vehicle_rgb_b)
        vehicle_rgb[((vehicle_rgb_r == 1) & (vehicle_rgb_b == 1) & (vehicle_rgb_g == 1))] = 255

        vehicle_rgb = opponent_region_of_interest(vehicle_rgb)
        self.Vehicle = vehicle_rgb

    def get_YellowLines(self,img,threshold = 100):

        # Lab Color Thresholding Yellow
        self._change_color_threshold((178,255))
        self._color_threshold(img,color_space = 'LAB',channel = 'L')
        yellow_Lab_L = self.current_color_mask

        self._change_color_threshold((1,255))
        self._color_threshold(img,color_space = 'LAB',channel = 'A')
        yellow_Lab_a = self.current_color_mask

        self._change_color_threshold((150,255))
        self._color_threshold(img,color_space = 'LAB',channel = 'B')
        yellow_Lab_b = self.current_color_mask

        yellow_Lab = np.zeros_like(yellow_Lab_L)
        yellow_Lab[(((yellow_Lab_L == 1) & (yellow_Lab_b == 1)))] = 1

        # HSV Color Thresholding Yellow
        self._change_color_threshold((10,90))
        self._color_threshold(img,color_space = 'HSV',channel = 'H')
        yellow_hsv_h = self.current_color_mask

        self._change_color_threshold((125,255))
        self._color_threshold(img,color_space = 'HSV',channel = 'S')
        yellow_hsv_s = self.current_color_mask

        self._change_color_threshold((165,255))
        self._color_threshold(img,color_space = 'HSV',channel = 'V')
        yellow_hsv_v = self.current_color_mask

        yellow_hsv = np.zeros_like(yellow_hsv_h)
        yellow_hsv[((yellow_hsv_h == 1) & (yellow_hsv_s == 1) & (yellow_hsv_v == 1))] = 1
        self.current_color_mask = yellow_hsv

        combined_yellow = np.zeros_like(yellow_hsv_h)
        combined_yellow[((yellow_Lab == 1) | (yellow_hsv == 1))] = 255
        combined_yellow = region_of_interest(combined_yellow)
        combined_yellow = perspective_transform(combined_yellow)

        self.contours_Filtering(combined_yellow,500)
        self.yellow_Lines = self.filtered

        nonzero = np.nonzero(self.yellow_Lines)

        if len(nonzero[0])  < threshold:
            self.yellow_detected = False
        else:
            self.yellow_detected = True

    def get_BlueLines(self,img, threshold = 100):

        # Lab Color Thresholding Blue
        self._change_color_threshold((110,255))
        self._color_threshold(img,color_space = 'LAB',channel = 'L')
        blue_Lab_L = self.current_color_mask

        self._change_color_threshold((0,255))
        self._color_threshold(img,color_space = 'LAB',channel = 'A')
        blue_Lab_a = self.current_color_mask

        self._change_color_threshold((0,100))
        self._color_threshold(img,color_space = 'LAB',channel = 'B')
        blue_Lab_b = self.current_color_mask

        blue_Lab = np.zeros_like(blue_Lab_b)
        blue_Lab[((blue_Lab_L == 1) & (blue_Lab_a == 1) & (blue_Lab_b == 1))] = 1

        # HSV Color Thresholding Blue
        self._change_color_threshold((135,180))
        self._color_threshold(img,color_space = 'HSV',channel = 'H')
        blue_hsv_h = self.current_color_mask

        self._change_color_threshold((130,255))
        self._color_threshold(img,color_space = 'HSV',channel = 'S')
        blue_hsv_s = self.current_color_mask

        self._change_color_threshold((178,255))
        self._color_threshold(img,color_space = 'HSV',channel = 'V')
        blue_hsv_v = self.current_color_mask

        blue_hsv = np.zeros_like(blue_hsv_h)
        blue_hsv[((blue_hsv_h == 1) & (blue_hsv_s == 1) & (blue_hsv_v == 1))] = 1
        self.current_color_mask = blue_hsv

        combined_blue = np.zeros_like(blue_hsv_h)
        combined_blue[((blue_Lab == 1) | (blue_hsv == 1))] = 255
        combined_blue = region_of_interest(combined_blue)
        combined_blue = perspective_transform(combined_blue)

        self.contours_Filtering(combined_blue,700)
        self.blue_Lines = self.filtered

        nonzero = np.nonzero(self.blue_Lines)

        if len(nonzero[0])  < threshold:
            self.blue_detected = False
        else:
            self.blue_detected = True

    def detect_Lines(self,img):
        self.get_BlueLines(img)
        self.get_YellowLines(img)

    def contours_Filtering(self,img,threshold = 100):
        img3 = img.astype(np.uint8)
        img2,contours,hie = cv2.findContours(img3,1,2)

        mask = np.ones(img3.shape, dtype="uint8") * 255

        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])

            if area < threshold:
                cv2.drawContours(mask, [contours[i]], -1, 0, -1)

        self.filtered = cv2.bitwise_and(img, img, mask=mask)

Edges = Edge()
