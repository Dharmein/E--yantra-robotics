
while robot.step(timestep) != -1:
    
    if first_time == 0:
        frame = camera.getImage(); 
        img = np.frombuffer(frame, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)) #img contains the image obtained from the camera, use this to write rest of your code
        first_time = first_time + 1
        frame = ""
        img = ""
        print("first",first_time)
    elif first_time == 1:
        
#        print("time ",timestep)        
        print("next")        
        #DO NOT MODIFY THE ABOVE 2 LINES
    ##############################################
    #Here write code you want to execute repeatedly
#    print("Image-",img)
#    print("image1-end")
        ref1=""
#        delay = 1000
        frame = camera.getImage(); 
        img = np.frombuffer(frame, np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)) #img contains the image obtained from the camera, use this to write rest of your code
    
        color = color_detection(ref1) #Remember to pass the necessary arguments as defined in the function definition 
        print("color output",color)
        ref2=""
        shape = shape_detection(ref2) #Remember to pass the necessary arguments as defined in the function definition 
        print("Shape output",shape)
#        timestep=-1
    #    pass
        move_epuck(shape,color)
        first_time = 0
        pass
        ###############################################



from controller import Robot, DistanceSensor, Motor
from controller import Camera
import cv2
import numpy

#controller variables
timestep = 32
MAX_SPEED = 6.28

robot = Robot()
camera = Camera('camera')
camera.enable(timestep)
height = camera.getHeight()
width = camera.getWidth()
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

def find_centroid(frame): #You can add or remove arguments to your liking
    '''
      # convert image to grayscale image
    This function takes the frame and returns the centroid of the detected contours
    '''
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # convert the grayscale image to binary image
    ret,thresh = cv2.threshold(gray_image,127,255,cv2.THRESH_BINARY_INV)
    
    # calculate moments of binary image
    M = cv2.moments(thresh)
    
    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
    
    print("cx",cX)
    print("cy",cY)
    return cX, cY

def apply_pid(centroid,avg_speed):#You can add or remove arguments to your liking
#    '''
#    This function takes the centroid and the average speed you want to drive the bot on and applies the P controller
#    concept to steer the bot appropriately 
#    '''
    print("hi")
    leftMotor.setVelocity(6.28) 	
    rightMotor.setVelocity(6.28)
#    centroid.step(50000)

    print("hi1")
#    leftMotor.setVelocity(0) 	
#    rightMotor.setVelocity(0)
#    robot.step(5000)
#    if cv2.waitKey(1) & 0xFF == ord('q'):
#        break
    
    return
    

while robot.step(timestep) != -1:
   
    data = camera.getImage() #DO NOT MAKE ANY CHANGE TO THIS LINE
    frame = numpy.frombuffer(data, numpy.uint8).reshape((camera.getHeight(), camera.getWidth(), 4)) #DO NOT MAKE ANY CHANGE TO THIS LINE
    #frame contains the image seen by the camera, use it to apply image processing concepts
    centroid = find_centroid(frame)
    apply_pid(centroid,4.71) 
    pass


