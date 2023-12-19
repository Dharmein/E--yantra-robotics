# Team ## 78
# Import Robot and Motor
from controller import Robot, Motor
import math
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
# time in [ms] of a simulation step
timestep = int(robot.getBasicTimeStep())
MAX_SPEED = 5.0

# initialize motors
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)
r_len = 0
r_angle = 0
r_speed = 0
r_step = 0

# Forward funtion
def go_straight(r_len):
    leftMotor.setVelocity(MAX_SPEED)
    rightMotor.setVelocity(MAX_SPEED)
    robot.step(r_len)

# Stop funtion
def stop_motor():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)

# Right funtion
def hard_right(r_angle):
    leftMotor.setVelocity(MAX_SPEED)   
    rightMotor.setVelocity(-MAX_SPEED)   
    robot.step(r_angle)

# Semi Circle funtion
def semi_circle(r_speed,r_step):
    leftMotor.setVelocity(MAX_SPEED)   
    rightMotor.setVelocity(r_speed*MAX_SPEED)   
    robot.step(r_step)

# Main para

# Straight - 450
go_straight(4500)

#stop
stop_motor()

#hard right
hard_right(1200)

#stop
stop_motor()

# Straight - 300
go_straight(3000)

#stop
stop_motor()

#semicircle1 - r-150
semi_circle(0.675,5550)

#stop
stop_motor()

#semicircle2 - r225
semi_circle(0.767,7600)

#stop
stop_motor()

#semicircle3 - r - 300
semi_circle(0.822,9800)

#stop
stop_motor()

#semicircle
semi_circle(0.665,5100)

#stop
stop_motor()

#forward = 400
go_straight(4100)

#stop
stop_motor()

pass

