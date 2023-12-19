
# Young Engineers_78
from controller import Robot, DistanceSensor, Motor
from controller import Camera
from vehicle import Driver
import cv2
import numpy
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# time in [ms] of a simulation step
TIMESTEP = 16
MAX_SPEED = 6.28

# initialize devices
leftMotor = robot.getMotor("left wheel motor")
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

dist = 0
set_dist = 0
bot_mode = 'WALL_FOLLOW'
err_Threshold = 15
ps6 = robot.getDistanceSensor('ps6')
ps6.enable(TIMESTEP)
bot_mode = "WALL_FOLLOW"
camera = Camera('camera')
camera.enable(TIMESTEP)
height = camera.getHeight()
width = camera.getWidth()
height_roi = int(height/6)
roi_y_start = int(height/4)
roi_y_end = roi_y_start + height_roi

# Main loop:
print('Starting..')
# count = 0
while robot.step(TIMESTEP) != -1:

    data = camera.getImage()
    frame = numpy.frombuffer(data, numpy.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4))

    sample = frame[roi_y_start, 0:width]
    [B, G, R, _] = sample.mean(axis=0)

    diff_thrash = 50
    
    if B - G > diff_thrash and B - R > diff_thrash:
        print("BLUE")
        leftMotor.setVelocity(MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        robot.step(500*TIMESTEP)

    if R - G > diff_thrash and R - B > diff_thrash:
        print("red zone")

        bot_mode = "LINE_FOLLOW"
        leftMotor.setVelocity(0.92*MAX_SPEED)
        rightMotor.setVelocity(MAX_SPEED)
        robot.step(100*TIMESTEP)
        # count += 1

    if bot_mode == "WALL_FOLLOW":
        psValue6 = ps6.getValue()
        dist_inv = psValue6
        dist = 255 - dist_inv
        set_dist = 120
        err = dist - set_dist
        v = 0.5*MAX_SPEED
        if err > err_Threshold:
            leftSpeed = v
            rightSpeed = MAX_SPEED
        elif err < err_Threshold:
            leftSpeed = MAX_SPEED
            rightSpeed = v
        else:
            leftSpeed = MAX_SPEED
            rightSpeed = MAX_SPEED

        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)

    else:
#        print("LF")
        data = camera.getImage()
        frame = numpy.frombuffer(data, numpy.uint8).reshape(
            (camera.getHeight(), camera.getWidth(), 4))

        crop_frame = frame[roi_y_start:roi_y_end, 0:width]
        
        gray = cv2.cvtColor(crop_frame, cv2.COLOR_BGR2GRAY)
        # print('Count:', count)
        # if count > 100:
        _, threshold = cv2.threshold(gray, 127, 255, 1)
        # else:
            # _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_TRUNC)
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # print("len contours", len(contours))
        if len(contours) > 0:  # if more than zero contours were found
            # Line may be found (Contour detected)
            # Estimate Error based on Centroid of the biggest contour
            max_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(threshold)
            if M["m00"] != 0:  # area not equal to zero
                cx = M["m10"] / M["m00"]
                print("cX", cx)
#                print("width", width)
                err = cx-(width / 2)
                print("Error", err)
                norm_err = err / (width/2)
            else:
                leftMotor.setVelocity(0)
                rightMotor.setVelocity(0)  # stop motors
        else:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)  # stop motors
        err_Threshold = 0.3
        if err > 0.2:
            print('Right')
            leftSpeed = MAX_SPEED
            rightSpeed = 0.4 * MAX_SPEED
        elif err < -err_Threshold:
            print('Left')
            leftSpeed = 0.4 * MAX_SPEED
            rightSpeed = MAX_SPEED
        else:
            print('Straight')
            leftSpeed = 0.75*MAX_SPEED
            rightSpeed = 0.75*MAX_SPEED

        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)