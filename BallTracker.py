from controller import Robot
from math import pi
import math
import cv2
import numpy as np

def nothing(x):
    pass

def deg2rad(deg):
    return deg / 180 * pi

def rad2deg(rad):
    return rad / pi * 180

#Robot and devices initialization
robot = Robot()

camera = robot.getDevice("right_camera")
head_pitch = robot.getDevice("head_pitch")
head_yaw = robot.getDevice("head_yaw")
head_pitch_sensor = robot.getDevice("head_pitch_sensor")
head_yaw_sensor = robot.getDevice("head_yaw_sensor")

timestep = int(robot.getBasicTimeStep())


#Enabling devices
camera.enable(timestep)
head_yaw_sensor.enable(timestep)
head_pitch_sensor.enable(timestep)


#initial head rotation rate of change
Y = -2*pi/120

while robot.step(timestep) != -1:
    camera_data = camera.getImage()
    frame = np.frombuffer(camera_data, np.uint8).reshape(camera.getHeight(), camera.getWidth(), 4)
    
    #Mask for detecting orange color
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    Mask = cv2.inRange(hsv, (15, 120, 150, 100), (125, 255, 255, 200))

    #Connected components
    output = cv2.connectedComponentsWithStats(Mask, 8, cv2.CV_32S)
    num_labels = output[0]
    labels = output[1]
    stats = output[2]
    objects = 0
    ball_loc = [camera.getWidth()/2, camera.getHeight()/2]
    for i in range(1, num_labels):
        t = stats[i, cv2.CC_STAT_TOP]
        l = stats[i, cv2.CC_STAT_LEFT]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        a = stats[i, cv2.CC_STAT_AREA]
    
    # Filter small, non-square objects
    if (1800 > a > 20 and (abs(h-w) < 5)) :
        objects += 1
        cv2.rectangle(frame, (l, t), (l+w, t+h), (255, 255, 120), 2)
        cv2.putText(frame, "ball", (l+w, t+h), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 0, 0), 2)
        ball_loc = [l+w/2, t-h/2]
    cv2.imshow("frame", frame)
    
    #Find ball to camera center difference
    center_point = [camera.getWidth()/2, camera.getHeight()/2]
    Diff = np.subtract(center_point, ball_loc)
    
    #If no objects detected then diff returns to zero
    if (objects == 0):
        Diff = [0, 0]
    
    #Center the ball to the camera frame (Horizontal)
    if (Diff[0] > 5 or -5 > Diff[0]) :
        X = head_yaw_sensor.getValue() - 0.05*Diff[0]*(pi/180)
        head_yaw.setPosition(X)
    elif (objects == 0) :
        #Sweep if no ball is detected
        if (abs(rad2deg(head_yaw_sensor.getValue()) - 90) < 1):
            Y = -2*pi/120
        elif (abs(rad2deg(head_yaw_sensor.getValue()) + 90) < 1):
            Y = 2*pi/120
        X = head_yaw_sensor.getValue() + Y
        head_yaw.setPosition(X)
    
    #Center the ball to the camera frame (Vertical)
    if (Diff[1] > 5 or -5 > Diff[1]) :
        X = head_pitch_sensor.getValue() + 0.4*Diff[1]*(pi/180)
        head_pitch.setPosition(-X)
    elif (objects == 0):
        head_pitch.setPosition(pi/8)
    
    #Press ESC to exit
    k = cv2.waitKey(15) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows