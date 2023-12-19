
#TeamId-78 Young Engineers
from controller import Robot, Motor, DistanceSensor
import math
robot = Robot()
timestep = 16
MAX_SPEED = 6.28
dist_sensor_max = 255
L = 52 # axle length in mm
R = 20 # wheel radius in mm
MAX_SPEED = 6.28
w= MAX_SPEED
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]
ps = []
for i in range(8):
    ps.append(robot.getDistanceSensor(psNames[i]))
    ps[i].enable(timestep)
def go_straight(w): 
    leftMotor.setVelocity(w)
    rightMotor.setVelocity(w)
def soft_right(w,t):
    leftMotor.setVelocity(w)
    rightMotor.setVelocity(0)
    robot.step(t)
def soft_left(w,t):
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(w)
    robot.step(t)   
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    psValues = []
    left_encoder = robot.getPositionSensor('left wheel sensor')
    left_encoder.enable(timestep)
    right_encoder = robot.getPositionSensor('right wheel sensor')
    right_encoder.enable(timestep)
    
    left_encoder_ticks = left_encoder.getValue()
    right_encoder_ticks = right_encoder.getValue()
   
    pi=3.14
   
    angle=(((left_encoder_ticks - right_encoder_ticks) * R / L) % (2*pi))*180/pi 
    
    print("Angle : ",angle)
    for i in range(8):
        psValues.append(ps[i].getValue())

    print("psValues[0] :", psValues[0], "psValues[7]:", psValues[7])
    if psValues[0] >= 45.0:
        soft_left(w,150)
        
    elif psValues[7]>= 45.0:
        soft_right(w,150)
        
    else:
        go_straight(w)
        
    if psValues[1] >= 45.0 or psValues[2]>= 30.0:
        soft_left(w,150)
        
    else:
        go_straight(w)
        
    if psValues[5] >= 60.0 or psValues[6]>= 60.0:
        soft_right(w,150)
        
    else:
        go_straight(w) 
    
    angle=(angle - 180) 
    
    if angle >= 0 :   
        soft_right(w,1)
   
    if angle <= 0.0:
        soft_left(w,1)
           
    pass #Add your sensing and acting code here