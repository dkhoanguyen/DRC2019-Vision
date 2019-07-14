from Utils import *
from path_planning import *

#calibration_coefficient = calibrate_camera()

frame = cv2.imread("image1.png")

warped = perspective_transform(frame)

Edges.detect_Lines(frame)

Edges._object_thresholding(frame)

Lane.get_YellowCoordinate(Edges.yellow_Lines,warped)

Lane.get_BlueCoordinate(Edges.blue_Lines,warped)

Lane.get_Center(warped)

Lane.pointVisulization(warped)

Objects.get_ObjectPoints(Edges.Objects,warped,6)

Paths.plan_Path()

Paths.show_Trajectory(warped)

print(Lane.blue_confident)

print(Objects.found)

# print(Objects.center)

# print(Edges.blue_detected)
# print(Edges.yellow_detected)

print(Paths.final_x)
print(Paths.final_y)
print((Paths.final_theta/pi)*180)
print(Paths.go_straight)

cv2.imshow("img",warped)
cv2.waitKey(0)