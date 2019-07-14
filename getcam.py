#!/usr/bin/env python2
import cv2
import rospy
import time
from std_msgs.msg import String,Int8,Int16MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import numpy as np
from path_planning import *
from math import *
import tf.transformations

class ControlNode(object):
    def __init__(self):
        self.odrive_publisher = None
        self.robotx = 0
        self.roboty = 0
        self.orientation = 0
        self.ocam_image = None

    def getRobotX(self):
        return self.robotx

    def getRobotY(self):
        return self.roboty

    def getOrientation(self):
        return self.orientation

    def getOcamImage(self):
        return self.ocam_image

    def setRobotX(self, robot_pos):
        self.robotx = robot_pos

    def setRobotY(self, robot_pos):
        self.roboty = robot_pos

    def setOrientation(self, orientation_val):
        self.orientation = orientation_val

    def setOcamImage(self, image):
        self.ocam_image = image

    def publishAngle(self, angle):
        pub = rospy.Publisher('angle', Int8, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        # while not rospy.is_shutdown():

        rospy.loginfo(angle)
        pub.publish(angle)
        rate.sleep()

    def odomcallback(self, data):
        self.setRobotX(data.pose.pose.position.x)
        self.setRobotY(data.pose.pose.position.y)

        #print("The robot pose is: " + str(self.robotx) + ", " + str(self.roboty))

        quaternion = (
        data.pose.pose.orientation.x,
        data.pose.pose.orientation.y,
        data.pose.pose.orientation.z,
        data.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        #print(euler)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        self.setOrientation(yaw)
        #print(self.orientation)
        #rospy.loginfo("The X, Y Position is: " + str(self.robotx) + ":" +str(self.roboty))
        #rospy.loginfo("The roll, pitch and yaw is: " + str(euler[0]) + ":" +str(euler[1]) + ":" +str(euler[2]))

    def callbackimage(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard an image")
        cvbridge = CvBridge()
        global ocam_image
        try:
            self.setOcamImage(cvbridge.imgmsg_to_cv2(data, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def listener(self):
        return

    def Lane_Detection(self, frame):

        warped = perspective_transform(frame)

        #warped = frame

        Edges.detect_Lines(frame)

        Edges._object_thresholding(frame)

        Lane.get_YellowCoordinate(Edges.yellow_Lines,warped)

        Lane.get_BlueCoordinate(Edges.blue_Lines,warped)

        Lane.get_Center(warped)

        Objects.get_ObjectPoints(Edges.Objects,warped,6)

        Edges.vehicle_Thresholding(frame)

        Vehicles.trackVehicle(Edges.Vehicle)

        Vehicles.stopNow()

        Paths.plan_Path()


        Paths.show_Trajectory(warped)


        print(Paths.final_x)
        print(Paths.final_y)
        print((Paths.final_theta/pi)*180)

        rimg = cv2.cvtColor(Edges.blue_Lines, cv2.COLOR_GRAY2BGR)
        limg = cv2.cvtColor(Edges.yellow_Lines, cv2.COLOR_GRAY2BGR)
        obimg = cv2.cvtColor(Edges.Objects, cv2.COLOR_GRAY2BGR)
        displaytop = np.hstack((obimg, rimg))
        displaybot = np.hstack((limg, warped))
        display = np.vstack((displaytop, displaybot))
        # cv2.imshow("img",frame)
        cv2.imshow("img",display)
        cv2.waitKey(1)

        print(Objects.found)
        return (Paths.final_x, Paths.final_y, Paths.final_theta)



    # def oldgotoXY(self, xtarget, ytarget, angletarget):
    #     motor = Int16MultiArray()
    #     startangle = self.orientation
    #     globalx = self.robotx + xtarget * cos(self.orientation)
    #     globaly = self.roboty + ytarget * sin(self.orientation)
    #     startx = self.robotx
    #     starty = self.roboty
    #     desiredDist = sqrt((xtarget)*(xtarget)+(ytarget)*(ytarget))
    #
    #     angle = atan(abs(xtarget)/abs(ytarget))
    #     if(desiredDist > 0.5 and desiredDist < 1):
    #         angle = angle *(1 + (1 - desiredDist) / 4)
    #     elif(desiredDist < 1):
    #         angle = angle * (1 + (1 - desiredDist))
    #
    #     else:
    #         angle = angle / 1.5
    #
    #     if(Paths.sharp_turn):
    #         angle = angle * 1.5
    #     else:
    #         angle = angle * 0.75
    #
    #     if(xtarget < 0):
    #         angle = -angle
    #
    #     basespeed = 7000
    #     angletune = 2300
    #     breakspeed = 9000#2000
    #     straightSpeed = 25000
    #
    #
    #
    #     if(xtarget < 0):
    #         motor0speed = basespeed - abs(angle) * angletune + desiredDist * breakspeed
    #         motor1speed = basespeed + abs(angle) * angletune + desiredDist * breakspeed
    #     else:
    #         motor0speed = basespeed + abs(angle) * angletune + desiredDist * breakspeed
    #         motor1speed = basespeed - abs(angle) * angletune + desiredDist * breakspeed
    #     if(Paths.go_straight):
    #         motor0speed = straightSpeed
    #         motor1speed = straightSpeed
    #     if(Objects.found > 0):
    #         motor0speed = motor0speed / 1.4
    #         motor1speed = motor1speed / 1.4
    #
    #     if(x < -40):
    #         motor1speed += 3000
    #     if(x > 40):
    #         motor0speed += 3000
    #     if(motor0speed > 32760):
    #         motor0speed = 32760
    #     if(motor1speed > 32760):
    #         motor1speed = 32760
    #
    #
    #     desiredAngle = startangle + angle
    #     motor.data=[int(-motor0speed), int(motor1speed)]
    #     rospy.loginfo("Motor Msg Being Sent is: "+ str(motor.data))
    #
    #     self.odrive_publisher.publish(motor)

    def gotoXY(self, xtarget, ytarget, angletarget): #go from X to Y
        motor = Int16MultiArray()
        startangle = self.getOrientation()
        globalx = self.getRobotX() + xtarget * cos(float(self.getOrientation()))
        globaly = self.getRobotY() + ytarget * sin(float(self.getOrientation()))
        startx = self.getRobotX()
        starty = self.getRobotY()
        desiredDist = sqrt((xtarget)*(xtarget)+(ytarget)*(ytarget))
        angle = atan(abs(xtarget)/abs(ytarget))


        if(desiredDist > 0.5 and desiredDist < 1): # medium distance
            angle = angle *(1 + (1 - desiredDist) / 7)
        elif(desiredDist < 1):
            angle = angle * (1 + (1 - desiredDist) / 4) #close distance
        else: # far distance
            angle = angle / 1

        if Objects.found == True:
            Paths.sharp_turn = True

        if(Paths.sharp_turn == True):
            angle = angle * 2.2
        else:
            angle = angle * 0.8



        if(xtarget < 0): #makes sure angle is the rigth direction
            angle = -angle

        basespeed = 10000 #minspeed
        angletune = 2600  #steering sharpness
        breakspeed = 6000  #if straight go this much faster
        straightSpeed = 29000 #
        objectslowration = 1.175
        sharpturnslow = 1.2



        if(xtarget < 0):
            motor0speed = basespeed - abs(angle) * angletune + desiredDist * breakspeed
            motor1speed = basespeed + abs(angle) * angletune + desiredDist * breakspeed
        else:
            motor0speed = basespeed + abs(angle) * angletune + desiredDist * breakspeed
            motor1speed = basespeed - abs(angle) * angletune + desiredDist * breakspeed
        if(Paths.go_straight):
            motor0speed = straightSpeed
            motor1speed = straightSpeed
        if(Objects.found > 0):
            motor0speed = motor0speed / objectslowration
            motor1speed = motor1speed / objectslowration
        if(Paths.sharp_turn):
            motor0speed = motor0speed / sharpturnslow
            motor1speed = motor1speed / sharpturnslow
        #
        # if(x < -40):
        #     motor1speed += 3000
        # if(x > 40):
        #     motor0speed += 3000

        if(motor0speed > 32760):
            motor0speed = 32760
        if(motor1speed > 32760):
            motor1speed = 32760

        if Objects.found == True or Paths.sharp_turn == True:
            go_back = True


        desiredAngle = startangle + angle
        motor.data=[int(-motor0speed), int(motor1speed)]
        rospy.loginfo("Motor Msg Being Sent is: "+ str(motor.data))

        self.odrive_publisher.publish(motor)

    def stop(self):
        motor = Int16MultiArray()
        motor.data=[0, 0]
        self.odrive_publisher.publish(motor)
        rospy.sleep(1.)

if __name__ == '__main__':
    control = ControlNode()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, control.callbackimage)
    rospy.Subscriber("/odrive/odom", Odometry, control.odomcallback)
    control.odrive_publisher = rospy.Publisher('odrive/set_vels', Int16MultiArray, queue_size=1)
    control.listener()
    #rate = rospy.Rate(60)
    rospy.sleep(1.)

    stopped = False
    while not rospy.is_shutdown():
        motor = Int16MultiArray()
        if control.getOcamImage() is not None:
            print("gotoab")
            #gotoXY(0.2,1,0)
            #otoXY(-0.2,1,0)

            (x,y,t) = control.Lane_Detection(control.ocam_image)
            print("Sharp Turn: " + str(Paths.sharp_turn))
            print("Straight: " + str(Paths.go_straight))
            #if(Paths.sharp_turn):
                #sharpTurn(x)
                #gotoAB(x,y,t)
                #rospy.sleep(2.)
            #else:
            if(Vehicles.stop or stopped):
                stopped = True
                control.stop()
            else:
                hello = 1
                control.gotoXY(x,y,t)


            #gotoXY(0.5,0.5,1)
            # (newx,newy) = getPoint1(x,y,t)
            # gotoXY(newx, newy,t)

            #motor.data=[-motor0speed, motor1speed]
            # self.odrive_publisher.publish(motor)
            # # gotoAB(x,y,t)

            #gotoAB(-0.2,1,0)
        #
        # rospy.sleep(0.05)
        #    print("no image")
        # speed = 8000
        # motor0speed = speed / 2
        # motor1speed = speed
        # motor = Int16MultiArray()

        # motor.data=[-motor0speed, motor1speed]
        # rospy.loginfo("Motor Msg Being Sent is: "+ str(motor.data))
        #
        # self.odrive_publisher.publish(motor)
        # rate.sleep()


    rospy.spin()
