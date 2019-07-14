import numpy as np
import cv2

def region_of_interest(img):

    height = img.shape[0]
    width = img.shape[1]

    region_height = height*0.2
    region_width1 = width*0.2
    region_width2 = width*0.8
    region_width3 = width*0.35
    region_width4 = width*0.65

    mask = np.zeros_like(img)
    region = np.array([[(0,height*0.85),(0,region_height),(width,region_height),(width,height*0.85)]],dtype=np.int32)

    cv2.fillPoly(mask,region,255)

    new_mask = cv2.bitwise_and(img,mask)

    return new_mask

def opponent_region_of_interest(img):

    height = img.shape[0]
    width = img.shape[1]

    region_height = height*0.2
    region_width1 = width*0.2
    region_width2 = width*0.8
    region_width3 = width*0.35
    region_width4 = width*0.65

    mask = np.zeros_like(img)
    region = np.array([[(0,height*0.8),(0,region_height),(width,region_height),(width,height*0.8)]],dtype=np.int32)

    cv2.fillPoly(mask,region,255)

    new_mask = cv2.bitwise_and(img,mask)

    return new_mask

def perspective_transform(img, reverse = False):

    height = img.shape[0]
    width = img.shape[1]

    src = np.float32([(0.325*width,180),
                  (0.675*width,180),
                  (0.0*width,425),
                  (1*width,425)])
    dst = np.float32([(200,int(height*0)),
                  (width - 200,int(height*0)),
                  (200,height),
                  (width - 200,height)])

    Normal = cv2.getPerspectiveTransform(src, dst)

    Reverse = cv2.getPerspectiveTransform(dst, src)

    if reverse == True:
        warped = cv2.warpPerspective(img, Reverse, (width,height), flags=cv2.INTER_LINEAR)

    else:

        warped = cv2.warpPerspective(img, Normal, (width,height), flags=cv2.INTER_LINEAR)

    return warped
