
#Young Engineers_78
from controller import Robot, DistanceSensor, Motor
from controller import Camera
from vehicle import Driver
import cv2
import numpy

TIMESTEP = 16 #timestep = int(driver.getBasicTimeStep())
MAX_SPEED = 6.28
robot = Robot()
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftSpeed = 0.0
rightSpeed = 0.0
leftMotor.setVelocity(leftSpeed)
rightMotor.setVelocity(rightSpeed)

def W_F():
    dist = 0
    set_dist = 0
    err = 0
    v = 0
    err_Threshold = 150
    ps6 = robot.getDistanceSensor('ps6')
    ps6.enable(TIMESTEP)
    
    # Main loop:
    while robot.step(TIMESTEP) != -1:
        psValue6 = ps6.getValue()
    
        dist_inv = psValue6
        dist = 255 - dist_inv
    
        set_dist = 100 
        err = dist - set_dist
        v = 0.6 * MAX_SPEED
     
        if   err > err_Threshold:
            leftSpeed = 0
            rightSpeed = v  
        elif err < err_Threshold:
            leftSpeed = MAX_SPEED
            rightSpeed = 0
        else:
            leftSpeed = MAX_SPEED
            rightSpeed = v
    
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed);
W_F();
def L_F():
    # Initialize Camera
    camera = Camera('camera')
    camera.enable(TIMESTEP)
    height = camera.getHeight()
    width = camera.getWidth()
    
    height_roi = height/6 
    
    roi_y_start = height/4
    roi_y_end = roi_y_start + height_roi
    
    norm_err = 0
    
    # feedback loop: step simulation until receiving an exit event
    while robot.step(TIMESTEP) != -1:
        data = camera.getImage();
        frame = numpy.frombuffer(data, numpy.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
        
        crop_frame = frame[roi_y_start: roi_y_end, 0:width]
        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(data, 120, 255, cv2.THRESH_TOZERO) 
        contours, _ = cv2.findContours(data, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
        if len() > 0: # if more than zero contours were found
            # Line may be found (Contour detected)
            # Estimate Error based on Centroid of the biggest contour
            max_controur = max(contours, key=cv2.contourArea)
            M = cv2.moments(c) 
            if M["??"] != 0: # area not equal to zero
                cx = int(M["m10"] / M["m00"])
                err = cx-(width /2)
                norm_err = err / (width/2)
            else:
                leftMotor.setVelocity(leftSpeed)
                rightMotor.setVelocity(rightSpeed)# stop motors
        else:
            leftMotor.setVelocity(leftSpeed)
            rightMotor.setVelocity(rightSpeed)# stop motors
        
        # P Control
        print ("normalized error: ", norm_err);
        kp = 0.1*MAX_SPEED
        
        # unicycle model
        v = 0.6 *MAX_SPEED
        w = kp*norm_err
        
        # differential drive model
        rightSpeed = MAX_SPEED
        leftSpeed  = MAX_SPEED 
    
        # Cap
        if leftSpeed > MAX_SPEED:
            leftSpeed = 0
        if rightSpeed > MAX_SPEED:
            rightSpeed = 0
            
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break;

