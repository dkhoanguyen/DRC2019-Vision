import cv2
import numpy as np
import matplotlib.pyplot as plt

def display_sidebyside(img1,img2):
    
    numpy_vertical = np.vstack((img1, img2))
    numpy_horizontal = np.hstack((img1, img2))

    numpy_vertical_concat = np.concatenate((img1, img2), axis=0)
    numpy_horizontal_concat = np.concatenate((img1, img2), axis=1)

    #cv2.imshow('Main', img1)
    #cv2.imshow('Numpy Vertical', numpy_vertical)
    #cv2.imshow('Numpy Horizontal', numpy_horizontal)
    #cv2.imshow('Numpy Vertical Concat', numpy_vertical_concat)
    cv2.imshow('Numpy Horizontal Concat', numpy_horizontal_concat)
