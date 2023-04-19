import math
import time
import numpy as np
# create a fuction take gpa as float and return a string with the gpa ?



integral_accumulator=0
prevVelocity=0
mode="ideal"
comefrom = "ideal"
prevangle=0
angle_accumulator=0
timer =time.time()

def state(img,averaged_lines,linearVelocity,leftLineOnlyExit,rightLineOnlyExit,twoLineExist): 
    global mode
    global timer
    global comefrom
    steering_angle=0
    throttle=0
    distfromrightline=0
    distfromleftline=0
    height, width, _ = img.shape
    mid = int((width / 2))
    print(mode)
    if(mode == "ideal"):

        if(leftLineOnlyExit ):
            
            mode="deviate_left"
        elif(rightLineOnlyExit):
            
            mode="deviate_right"
        angle = errorAngle(img,averaged_lines, twoLineExist) 
        steering_angle=steer_PID_Controller(angle)
 
        throttle =velocity_PID_Controller(linearVelocity, 3, 0.5,0.1,0.01)


    elif(mode == "deviate_left") :

        if(twoLineExist):
            mode="ideal"
        elif(linearVelocity<0.2):
            mode="stuck"


        steering_angle = -11 * np.pi/180
        throttle =velocity_PID_Controller(linearVelocity, 2, 0.7,0.7,0.01)  


    elif(mode == "deviate_right"):
        if(twoLineExist):
            mode="ideal"
        elif(linearVelocity<0.2):
            mode="stuck"
  


        steering_angle = 11 * np.pi/180
        throttle =velocity_PID_Controller(linearVelocity, 2, 0.7,0.7,0.01)

    elif(mode == "stop"):
        throttle = velocity_PID_Controller( linearVelocity, 0, 0.7,0.7,0.01)
        if(linearVelocity<0.2):
            if(leftLineOnlyExit):
                mode="deviate_left"
            elif(rightLineOnlyExit):
                mode="deviate_right"
            elif(twoLineExist):
                mode="ideal"

    elif(mode == "reverse"):

        throttle = velocity_PID_Controller( linearVelocity, -1 , 0.7,0.7,0.01 )
        if(time.time()-timer>5):
            mode="stop"

    elif(mode == "stuck"):
        mode="reverse"

        # throttle =velocity_PID_Controller(linearVelocity, 1.5)
        
    return steering_angle, throttle

def errorAngle (img,averaged_lines,twoLineExist):
    height, width, _ = img.shape
    mid = int((width / 2)) 
    angle_to_mid_radian=0   
    if twoLineExist :
        left_x2= averaged_lines[0][2]
        right_x2= averaged_lines[1][2]
        left_x1=averaged_lines[0][0]
        right_x1=averaged_lines[1][0]
        x_offset_upper = (left_x2 + right_x2) / 2 - mid
        x_offset_lower = (left_x1 + right_x1) / 2 - mid
        
        y_offset =height-70 - 300
        angle_to_mid_radian = -1*math.atan( x_offset_upper / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line

    return angle_to_mid_radian

def steer_PID_Controller(angleToMid):

    global prevangle
    global angle_accumulator
    Kp=0.1
    kd=0.15
    ki=0.01
    #print(angleToMid)

    # if angleToMid <-50 or angleToMid > 50:
    #      SteeringAngle = 0
    #      return SteeringAngle
     

    error = angleToMid
    if error < 0.1 :
        angle_accumulator=0

    angle_accumulator += error

    proportional_part=error*Kp
    derviative_part=(error-prevangle)*kd
    integral_part= angle_accumulator*ki
    prevangle=error

    SteeringAngle=proportional_part+integral_part +derviative_part

    return SteeringAngle
def velocity_PID_Controller(linearVelocity,desiredVelocity,Kp,kd,ki):

    global prevVelocity
    global integral_accumulator

    #print(angleToMid)

    # if angleToMid <-50 or angleToMid > 50:
    #      SteeringAngle = 0
    #      return SteeringAngle
     

    velocityError = desiredVelocity - linearVelocity
    if velocityError < 0.1 :
        integral_accumulator=0

    integral_accumulator += velocityError

    proportional_part=velocityError*Kp
    derviative_part=(velocityError-prevVelocity)*kd
    integral_part= integral_accumulator*ki
    prevVelocity=velocityError

    throttle=np.tanh(proportional_part+integral_part +derviative_part)

    return throttle
    

