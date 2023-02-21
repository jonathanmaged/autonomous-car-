import time

# pylint: disable=import-error
import cv2
import numpy as np
import matplotlib.pyplot as plt
from machathon_judge import Simulator, Judge
import keyboard

class FPSCounter:
    def __init__(self):
        self.frames = []

    def step(self):
        self.frames.append(time.monotonic())

    def get_fps(self):
        n_seconds = 5

        count = 0
        cur_time = time.monotonic()
        for f in self.frames:
            if cur_time - f < n_seconds:  # Count frames in the past n_seconds
                count += 1

        return count / n_seconds


#Thresholding, warping , summation of pixels, averaging, display
def make_coordinates(img,line_parameters):
    print(line_parameters)
    if line_parameters is not None:
        try:
            slope, intercept = line_parameters
        except TypeError:
            slope, intercept = 0.001, 0
        y1=img.shape[0]
        y2=int(y1*(3/5))
        x1=int((y1-intercept)/slope)
        x2=int((y2-intercept)/slope)
    return np.array([x1,y1,x2,y2])

def average_slope_intercept(img, lines):
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
    left_fit_average= np.average(left_fit,axis=0)
    right_fit_average= np.average(right_fit,axis=0) 
    left_line=make_coordinates(img,left_fit_average)
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
            cv2.line(line_image,(x1,y1),(x2,y2),(0,0,255),5)
    return line_image     

def laneDetection(img):
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    masked = cv2.bitwise_and(hsv_frame, img, mask=red_mask)
    Canny = canny(red_mask)
   # cv2.imshow("canny",Canny)
    cropped_image= region_of_interest(Canny)
   # cv2.imshow("cropped_image",cropped_image)
    lines= cv2.HoughLinesP(cropped_image,2,np.pi/180,100,np.array([]),minLineLength=40,maxLineGap=5)
    averaged_lines= average_slope_intercept(img,lines)
    line_image=display_lines(img,averaged_lines)
   # cv2.imshow("lines",line_image)
    combo_image=cv2.addWeighted(img,0.8,line_image,1,1)
    cv2.imshow("test",combo_image)
    return img

def run_car(simulator: Simulator) -> None:
    """
        simulator : Simulator
        The simulator object to control the car
        The only functions that should be used are:
        - get_image()
        - set_car_steering()
        - set_car_velocity()
        - get_state()
    """
    fps_counter.step()

    # Get the image and show it
    img = simulator.get_image()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    fps = fps_counter.get_fps()

    # draw fps on image
    cv2.putText(
        img,
        f"FPS: {fps:.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.imshow("Live", img)
    cv2.waitKey(1)

    laneDetection(img)
    
    # Control the car using keyboard
    steering = 0
    if keyboard.is_pressed("a"):
        steering = 1
    elif keyboard.is_pressed("d"):
        steering = -1

    throttle = 0
    if keyboard.is_pressed("w"):
        throttle = 1
    elif keyboard.is_pressed("s"):
        throttle = -1
        

    simulator.set_car_steering(steering * simulator.max_steer_angle / 1.7)
    simulator.set_car_velocity(throttle * 25)


if __name__ == "__main__":
    # Initialize any variables needed
    cv2.namedWindow("Live", cv2.WINDOW_NORMAL)
    fps_counter = FPSCounter()

    # You should modify the value of the parameters to the judge constructor
    # according to your team's info
    judge = Judge(team_code="your_new_team_code", zip_file_path="your_solution.zip")

    # Pass the function that contains your main solution to the judge
    judge.set_run_hook(run_car)

    # Start the judge and simulation
    judge.run(send_score=False, verbose=True)
