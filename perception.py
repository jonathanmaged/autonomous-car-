import math
import time

# pylint: disable=import-error
import cv2
import numpy as np
import matplotlib.pyplot as plt
import keyboard


twoLineExist=False
rightLineOnlyExit=False
leftLineOnlyExit=False

def make_coordinates(img,line_parameters):
    height, width, _ = img.shape
    if line_parameters is not None:
        slope, intercept = line_parameters
        #print(slope)
        if slope==0:
            slope=0.000000000001


        y1=height-70
        y2=300

        x1=max(-2*width,min(2*width,int((y1-intercept)/slope)))
        x2=max(-2*width,min(2*width,int((y2-intercept)/slope)))

    return np.array([x1,y1,x2,y2])

def average_slope_intercept(img, lines):
    global twoLineExist
    global rightLineOnlyExit
    global leftLineOnlyExit
    left_fit=[]
    right_fit=[] 

    if lines is not None:  
        for line in lines:
            x1,y1,x2,y2=line.reshape(4)  
            parameters= np.polyfit((x1,x2),(y1,y2),1)  
            slope=parameters[0]
            intercept=parameters[1]
            if slope < -0.3 and slope>-1.5 :
                left_fit.append((slope,intercept))
            elif slope>0.3 and slope<1.5:
                right_fit.append((slope,intercept))

    if  left_fit and right_fit:
        twoLineExist =True 
        rightLineOnlyExit=False
        leftLineOnlyExit=False
        left_fit_average= np.average(left_fit,axis=0)
        left_line=make_coordinates(img,left_fit_average)
        right_fit_average= np.average(right_fit,axis=0) 
        right_line=make_coordinates(img,right_fit_average)
        return np.array([left_line,right_line])
    elif left_fit :
        twoLineExist=False
        leftLineOnlyExit=True
        left_fit_average= np.average(left_fit,axis=0)
        left_line=make_coordinates(img,left_fit_average)
        return np.array([left_line])
    elif right_fit:
        twoLineExist=False
        rightLineOnlyExit=True
        right_fit_average= np.average(right_fit,axis=0) 
        right_line=make_coordinates(img,right_fit_average)
        return np.array([right_line])
    else:
        twoLineExist =False
        leftLineOnlyExit=False
        rightLineOnlyExit=False 
        return
        
def canny(img):
    #gray=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
    blur= cv2.GaussianBlur(img,(5,5),0)
    canny=cv2.Canny(blur,50,150)
    return canny

def region_of_interest(img):
    h= img.shape[0]
    polygon = np.array([
    [(0,h-70),(640,h-70),(600,150),(80,150)]
    ])
    mask=np.zeros_like(img)
    cv2.fillPoly(mask,polygon,255)
    # cv2.imshow("cred_image",mask)
    masked_img = cv2.bitwise_and(img,mask)
    return masked_img

def display_lines(img,lines):
    line_image=np.zeros_like(img)
    
    if lines is not None:
        for x1,y1,x2,y2 in lines:    
            #print(int(x1),int(y1),int(x2),int(y2))   
            cv2.line(line_image,(int(x1),int(y1)),(int(x2),int(y2)),(0,0,255),5)
    return line_image     

def perspective_warp(image):

    h, w , _ = image.shape
    source = np.float32([[10, 219],
    [600, 219],
    [10, 449],
    [600, 449]])
    # plt.imshow(image)
    # plt.show()
    destination = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(source, destination)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(image, M, (image.shape[1], image.shape[0]))
    return warped

def perception(img):
    # cv2.imshow("original",img)
    steering_lines=np.zeros_like(img)
    height, width, _ = img.shape


    img=perspective_warp(img)
    # plt.imshow(img)
    # plt.show()
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
    
    #warp_image=perspective_warp(Canny)
    cv2.imshow("cropped_image",cropped_image)
   

    lines= cv2.HoughLinesP(cropped_image,2,np.pi/150,50,minLineLength=10,maxLineGap=1)
    averaged_lines= average_slope_intercept(img,lines)
    line_image=display_lines(img,averaged_lines)
    combo_image=cv2.addWeighted(img,0.8,line_image,1,1)

    #mid line of the camera in the middle of the simulation 
    mid_line= cv2.line(steering_lines,(int((width / 2)),int(height)),(int((width / 2)),int(height/2)),(255,0,0),2)
    combo_image=cv2.addWeighted(combo_image,0.8,mid_line,1,1)
    ##############################

    #mid line of the lane which is used to calculate the offset between it and the mid line
    if  twoLineExist:
        left_x2= averaged_lines[0][2]
        right_x2= averaged_lines[1][2]
        left_x1=averaged_lines[0][0]
        right_x1=averaged_lines[1][0]
        x_upper = int((left_x2 + right_x2) / 2)
        x_lower = int((left_x1 + right_x1) / 2 )
        mid_lane_segment_upper=cv2.line(steering_lines,(x_upper,300-10),(x_upper,300+10),(0,255,0),2)
        mid_lane_segment_lower=cv2.line(steering_lines,(x_lower,int(height-70-10)),(x_lower,int(height-70+10)),(0,255,0),2)
        combo_image=cv2.addWeighted(combo_image,0.8,mid_lane_segment_upper,1,1)
        combo_image=cv2.addWeighted(combo_image,0.8,mid_lane_segment_lower,1,1)
    ###############################

    color = (0,255, 0)  
    thickness = 2
    if(twoLineExist):
        for x1, y1, x2, y2 in averaged_lines:    
            cv2.circle(combo_image, (x1,y1 ), 5, color, thickness)
            cv2.circle(combo_image, (x2,y2 ), 5, color, thickness)
    return combo_image,averaged_lines ,leftLineOnlyExit ,rightLineOnlyExit,twoLineExist