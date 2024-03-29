# =============================================================================
#Created on Mon Jun 17 12:23:15 2019
#
#@author: Mahasvin
# =============================================================================


# import the necessary packages
import numpy as np
import cv2

# initialize the list of reference points and boolean indicating
# whether cropping is being performed or not
x_start, y_start, x_end, y_end = 0, 0, 0, 0
cropping = False
getROI = False
refPt = []
lower = np.array([])
upper = np.array([])

green = np.uint8([[[26, 83, 255]]])
hsvGreen = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
lowerLimit = (hsvGreen[0][0][0]-10,100,100)
upperLimit = (hsvGreen[0][0][0]+10,255,255)

camera = cv2.VideoCapture(0)


def click_and_crop(event, x, y, flags, param):
    # grab references to the global variables
    global x_start, y_start, x_end, y_end, cropping, getROI

    # if the left mouse button was clicked, record the starting
    # (x, y) coordinates and indicate that cropping is being
    # performed
    if event == cv2.EVENT_LBUTTONDOWN:
        x_start, y_start, x_end, y_end = x, y, x, y
        cropping = True

    elif event == cv2.EVENT_MOUSEMOVE:
        if cropping == True:
            x_end, y_end = x, y

    # check to see if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        # record the ending (x, y) coordinates and indicate that
        # the cropping operation is finished
        x_end, y_end = x, y
        cropping = False
        getROI = True


cv2.namedWindow("image")
cv2.setMouseCallback("image", click_and_crop)


# keep looping
while True:

    if not getROI:

        # get one of the following tags: "blue", "yellow", "red"
        print("[+] Enter the tag for the color you are going to select (blue, yellow, red):")
        tag = str(input())
        # check if the entered tag is spelled right
        while (tag != "blue") and (tag != "yellow") and (tag != "red"):
            print("[+] ERROR: The entered tag does not match blue, yellow or red! Try again:")
            tag = str(input())

        while True:
            # grab the current frame
            (grabbed, frame) = camera.read()

            if not grabbed:
                break

            if not cropping and not getROI:
                cv2.imshow("image", frame)

            elif cropping and not getROI:
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
                cv2.imshow("image", frame)

            elif not cropping and getROI:
                cv2.rectangle(frame, (x_start, y_start), (x_end, y_end), (0, 255, 0), 2)
                cv2.imshow("image", frame)
                break

            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                noROI = True
                break

        # if there are two reference points, then crop the region of interest
        # from teh image and display it
        refPt = [(x_start, y_start), (x_end, y_end)]

        roi = frame[refPt[0][1]:refPt[1][1], refPt[0][0]:refPt[1][0]]
        #cv2.imshow("ROI", roi)

        hsvRoi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        print('[+] ' + tag + ': min H = {} , minS S = {} , min V = {} ; max H = {} , max S = {} , max V = {} '.format(hsvRoi[:,:,0].min(), hsvRoi[:,:,1].min(), hsvRoi[:,:,2].min(), hsvRoi[:,:,0].max(), hsvRoi[:,:,1].max(), hsvRoi[:,:,2].max()))

        lower = np.array([hsvRoi[:,:,0].min(), hsvRoi[:,:,1].min(), hsvRoi[:,:,2].min()])
        upper = np.array([hsvRoi[:,:,0].max(), hsvRoi[:,:,1].max(), hsvRoi[:,:,2].max()])

        flag0=0
        flag1=0
        flag2=0
        flag3=0
        flagblue=0
        flagyellow=0

        for item in lower:
            if item < 25:
                flag0=1
            elif item ==60:
                flag1=1

        for item in upper:
            if item==60:
                flag2=1
            elif item > 120:
                flag3=1

        if flag0==1 and flag2==1:
            flagyellow=1
        if flag1==1 and flag3==1:
            flagblue=1


            #lower = np.array(lower, dtype = np.uint8)
            #upper = np.array(upper, dtype = np.uint8)

        if list(lower)<= [25,135,170 -10,100,100] and list(upper)>=[59,250,250] and flagyellow==1:
            print ("yellow")
        elif list(lower)<= [70,135,150] and list(upper)>=[105,250,250] and flagblue==1:
            print ("blue")

        # output values
        output_values = ' : min H = {} , min S = {} , min V = {} ; max H = {} , max S = {} , max V = {} '.format(hsvRoi[:,:,0].min(), hsvRoi[:,:,1].min(), hsvRoi[:,:,2].min(), hsvRoi[:,:,0].max(), hsvRoi[:,:,1].max(), hsvRoi[:,:,2].max()) + '\n'


        ## store values
        # check if a file already exists; if so, change only the values for the color stored in the [tag] variable
        try:
            with open('HSVcolor.txt', 'r') as file:
                data = file.readlines()
            for (i,line) in enumerate(data):
                if tag not in line:
                    with open('HSVcolor.txt', 'a+') as file:
                        file.write(tag + output_values)
                elif tag in line:
                    data[i] = tag + output_values
                    with open('HSVcolor.txt', 'w') as file:
                        file.writelines(data)

        # if there exists no HSVcolor.txt file, then create a new one and store value
        except:
            print("line 1")
            with open('HSVcolor.txt', 'w') as file:
                file.write(tag + output_values)

    # grab the current frame
    (grabbed, frame) = camera.read()

    if not grabbed:
        break

    # resize the frame, blur it, and convert it to the HSV
    # color space
    #frame = imutils.resize(frame, width=600)

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color from dictionary`1, then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    kernel = np.ones((9,9),np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # only proceed if the radius meets a minimum size. Correct this value if obect's size is smaller.
        if radius > 0.5:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.putText(frame, 'center: {}, {}'.format(int(x), int(y)), (int(x-radius),int(y-radius)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

    # show the frame to our screen
    cv2.imshow("image", frame)


    key = cv2.waitKey(1) & 0xFF
    # if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break
    elif key == ord("r"):
        getROI = False

# cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()
