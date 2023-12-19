from controller import Robot, Motor
from controller import Camera
import cv2
import numpy as np
import math

MAX_SPEED = 6.28
w = MAX_SPEED
#w = 0.75*MAX_SPEED #You can tweak it if you want
#w = 5
R = 20.5 # In mm
n=len
robot = Robot()
#timestep = 32 
timestep = int(robot.getBasicTimeStep())
steps=0
first_time = 0
camera = Camera('camera') #Use this to get image from the camera using getImage Function
camera.enable(timestep)

leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)
ref1=""
    
def color_detection(ref1): #Remember to put actual argument names in place  of arg1 arg2, you can have more or less than 2 arguments

#Write your color detection code here       
# detecting green color
#    delay = 100
    kernel = np.ones((5,5),np.uint8)
    dilation = cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel)
    img_hsv=cv2.cvtColor(dilation, cv2.COLOR_BGR2HSV)
#    print("Entry1")
    lower_green = np.array([45, 150, 50])
    upper_green = np.array([65, 255, 255])
    mask_green=cv2.inRange(img_hsv, lower_green, upper_green)
    res_green=cv2.bitwise_and(img,img, mask=mask_green)
    contours, hierarchy = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#    print("Green Contour starts",contours)
#    print("Green image",img)
    for contour in contours:
            ref1="Green"
#            print("Ref1 Green",ref1)
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2) 
                  
                cv2.putText(img, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 243, 0))
                return ref1

#    cv2.imshow("HSV",img_hsv)
#    print("HSV",img_hsv,"HSV end")
#    cv2.imshow("Multiple Color Detection", imageFrame) 
#    print("imageFrame",imageFrame)
#    while(True):
#       print("true check") 
#       k = cv2.waitKey(5) & 0xFF
#       print("k",k)
#       if k == 27:
#       if k == 255:
#          return ref1
#    print("entry2")
#    cv2.destroyAllWindows()
#    print("entry3")
#    return ref1     

#    if ref1=="Green":
# detecting blue color
#    kernel = np.ones((5,5),np.uint8)
#    dilation = cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel)
#    img_hsv=cv2.cvtColor(dilation, cv2.COLOR_BGR2HSV)
#    print("EntryBlue")
    lower_blue = np.array([115, 150, 0])
    upper_blue = np.array([135, 255, 255])
    mask_blue=cv2.inRange(img_hsv, lower_blue, upper_blue)
#    print("Mask_Blue",mask_blue)
    res_blue=cv2.bitwise_and(img,img, mask=mask_blue)
#    print("Res_Blue",res_blue)
    contours, hierarchy = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#    print("Blue Contour starts",contours)
#    print("Image Blue",img)
    for contour in contours: 
            ref1="Blue"
#            print("Ref1 Blue",ref1)
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                imageFrame = cv2.rectangle(img, (x, y), (x + w, y + h), (255, 243, 0), 2) 
                  
                cv2.putText(img, "Blue Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0))
                return ref1

#    cv2.imshow("HSV",img_hsv)
#    print("HSV1",img_hsv,"HSV1 end")
#    cv2.imshow("Multiple Color Detection", imageFrame) 
 #   print("imageFrame1",imageFrame)
 #   while(True):
 #      print("true check1") 
 #      k = cv2.waitKey(5) & 0xFF
 #      print("k",k)
#       if k == 27:
#       if k == 255:
#          return ref1
#    print("entry21")
#    cv2.destroyAllWindows()
#    print("entry31")
#    return ref1     

def shape_detection(ref2): #Remember to put actual argument names in place  of arg1 arg2, you can have more or less  than 2 arguments
#"""
#Write your shape detection code here
#"""            
#    print("shapre detection",n)
#    delay = 100
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    kernel = np.ones((3,3),np.uint8)
    morph = cv2.morphologyEx(gray,cv2.MORPH_CLOSE,kernel) 
    
     threshold = cv2.threshold(morph, 200, 255, cv2.THRESH_BINARY_INV)
#    print("threshold",threshold)    
    contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    pentagon_hit = 0
    
#    print("contours",contours)
    font = cv2.FONT_HERSHEY_COMPLEX
#    print("s1")
    for cnt in contours:
#        print("Shape1")
        approx = cv2.approxPolyDP(cnt, 0.025*cv2.arcLength(cnt, True), True)
#        print("approx",approx)
        cv2.drawContours(img, [approx], 0, (0, 255, 0))
        x = approx.ravel()[0]
        y = approx.ravel()[1]
#        print("Len-approx",len(approx))
    if len(approx) == 3:
            cv2.putText(img, "Triangle", (x, y), font, 0.5, (0, 0, 255))
            print("Triangle")
            ref2="Triangle"
    elif len(approx) == 4:
            cv2.putText(img, "Rectangle", (x, y), font, 0.5, (0, 0, 255))
            print("Rectangle")
            ref2="Rectangle"
    elif len(approx) == 5:
            cv2.putText(img, "Pentagon", (x, y), font, 0.5, (0, 0, 255))
            print("Pentagon")
            ref2="Pentagon"
            pentagon_hit = pentagon_hit + 1
#        elif len(approx) == 6:
#            cv2.putText(img, "Hexagon", (x, y), font, 0.5, (0, 0, 255))
#            print("Hexagon")
#            ref2="Hexagon"
#        elif len(approx) == 7:
#            cv2.putText(img, "Heptagon", (x, y), font, 0.5, (0, 0, 255))
#            print("Heptagon")
#            ref2="Heptagon"
        elif 6 <= len(approx) < 15:
            cv2.putText(img, "Circle", (x, y), font, 0.5, (0, 0, 255))
            print("Circle")
            ref2="Circle"
             

#        ref2=cv2.putText(img)
#        print(cv2.putText(img))   
#    print(ref2)
    print("pentagon hit",pentagon_hit)
    if pentagon_hit == 0:
        return ref2   
    elif pentagon_hit == 1:
        return "Pentagon"
    elif pentagon_hit == 2:
        return "Circle"  
    else:
        return ref2        
 #       img=cv2.resize(img,(600,600))     
 #       cv2.imshow("shapes", img)
 #       threshold=cv2.resize(threshold,(600,600))
 #       cv2.imshow("Threshold", threshold)
 #       cv2.waitKey(0)
 #       cv2.destroyAllWindows()
        
        
def move_epuck(shape,color): #You may have different arguments in your code
#"""
#Write your actuator code here
#"""  
#    delay = 100
    if shape == "Circle":
        steps=2290
    elif shape == "Triangle":
        steps=3428              
    elif shape == "Rectangle":
        steps=4570              
    elif shape == "Pentagon":
        steps=5713    
    else:
        steps=0           

    if color == "Blue":
        print("Turn left")
        leftMotor.setVelocity(-w)
        rightMotor.setVelocity(w)
        robot.step(2090)
        print("L1")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

    else:
        print("Turn right")
        leftMotor.setVelocity(w)
        rightMotor.setVelocity(-w)
        robot.step(2090)
        print("R1")
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(0)

    leftMotor.setVelocity(w)
    rightMotor.setVelocity(w)
    robot.step(steps)
    
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)


#    leftMotor.setVelocity(w)
#    rightMotor.setVelocity(w)
#    if n == w:
#       leftMotor.setVelocity(0)
#       rightMotor.setVelocity(6.28)
          
                          
#    return    
            
            
