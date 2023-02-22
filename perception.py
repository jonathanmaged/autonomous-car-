import math
import time

# pylint: disable=import-error
import cv2
import numpy as np
import matplotlib.pyplot as plt
import keyboard


def make_coordinates(img,line_parameters):
    height, width, _ = img.shape
    if line_parameters is not None:
        try:
            slope, intercept = line_parameters
        except TypeError:
            slope, intercept = 0.001, 0
        if slope==0:
            slope=0.000000000001
        y1=height
        y2=int(height*(3.5/5))
        try:
            x1=max(-2*width,min(2*width,int((y1-intercept)/slope)))
            x2=max(-2*width,min(2*width,int((y2-intercept)/slope)))
        except UnboundLocalError :
            print(x1,x2)

    return np.array([x1,y1,x2,y2])

def average_slope_intercept(img, lines):
    global oneLineExist ,rightLineOnlyExit,leftLineOnlyExit
    left_fit=[]
    right_fit=[] 
    if lines is not None:  
        for line in lines:
            x1,y1,x2,y2=line.reshape(4)  
            parameters= np.polyfit((x1,x2),(y1,y2),1)  
            slope=parameters[0]
            intercept=parameters[1]
            if slope < 0 :
                left_fit.append((slope,intercept))
            else:
                right_fit.append((slope,intercept))
    #left_line=np.array([0,0,0,0])
    #right_line=np.array([0,0,0,0])
    if  left_fit and right_fit:
        oneLineExist =False 
        rightLineOnlyExit=False
        leftLineOnlyExit=False
    elif left_fit :
        oneLineExist=True
        leftLineOnlyExit=True
    elif right_fit:
        oneLineExist=True
        rightLineOnlyExit=True
    else:
        oneLineExist =False
        leftLineOnlyExit=False
        rightLineOnlyExit=False 
       
    left_fit_average= np.average(left_fit,axis=0)
    left_line=make_coordinates(img,left_fit_average)
    right_fit_average= np.average(right_fit,axis=0) 
    right_line=make_coordinates(img,right_fit_average)
    return np.array([left_line,right_line])


def canny(img):
    #gray=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    blur= cv2.GaussianBlur(img,(5,5),0)
    canny=cv2.Canny(blur,50,150)
    return canny

def region_of_interest(img):
    h= img.shape[0]
    polygon = np.array([
    [(0,h),(640,h),(640,360),(350,288),(0,350)]
    ])
    mask=np.zeros_like(img)
    cv2.fillPoly(mask,polygon,255)
    masked_img = cv2.bitwise_and(img,mask)
    return masked_img

def display_lines(img,lines):
    line_image=np.zeros_like(img)
    
    if lines is not None:
        for x1,y1,x2,y2 in lines:    
            print(int(x1),int(y1),int(x2),int(y2))   
            cv2.line(line_image,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),5)
    return line_image     

def perception(img):
    steering_lines=np.zeros_like(img)
    height, width, _ = img.shape
    #as the lane in the simulation is red so the following approch is used 
    #this approch get all the red coloured object in th image
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    masked = cv2.bitwise_and(hsv_frame, img, mask=red_mask)
    ###############################

    #canny is used to make edge detection
    Canny = canny(red_mask)
    ###############################

    cropped_image= region_of_interest(Canny)
    lines= cv2.HoughLinesP(cropped_image,2,np.pi/180,100,np.array([]),minLineLength=40,maxLineGap=5)
    averaged_lines= average_slope_intercept(img,lines)
    line_image=display_lines(img,averaged_lines)
    combo_image=cv2.addWeighted(img,0.8,line_image,1,1)

    #mid line of the camera in the middle of the simulation 
    mid_line= cv2.line(steering_lines,(int((width / 2)),int(height)),(int((width / 2)),int(height/2)),(255,0,0),2)
    combo_image=cv2.addWeighted(combo_image,0.8,mid_line,1,1)
    ##############################

    #mid line of the lane which is used to calculate the offset between it and the mid line
    left_x2= averaged_lines[0][2]
    right_x2= averaged_lines[1][2]
    x = int((left_x2 + right_x2) / 2)
    mid_lane_segment=cv2.line(steering_lines,(x,int(height*(3.5/5))-10),(x,int(height*(3.5/5))+10),(0,255,0),2)
    combo_image=cv2.addWeighted(combo_image,0.8,mid_lane_segment,1,1)
    ###############################
    return combo_image,averaged_lines