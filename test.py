import math
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

def stabilize_steering_angle(
          curr_steering_angle, 
          new_steering_angle, 
          num_of_lane_lines, 
          max_angle_deviation_two_lines=5, 
          max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    if new angle is too different from current angle, 
    only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
            + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    return stabilized_steering_angle

def control(img,averaged_lines):
    height, width, _ = img.shape

    steeringRatio=0
    throttleRatio=0

    if leftLineOnlyExit :
        steeringRatio = -1
        throttleRatio=0.2
        numOFLine=1

    elif rightLineOnlyExit:
        steeringRatio = 1 
        throttleRatio=0.2
        numOFLine=1
    else :
        numOFLine=2
        left_x2= averaged_lines[0][2]
        right_x2= averaged_lines[1][2]
        mid = int((width / 2))
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height*(3.5/5))
        angle_to_mid_radian = math.atan( x_offset / y_offset)  # angle (in radian) to center vertical line
        angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
        steering_angle = angle_to_mid_deg 
        #stabilized_angle = stabilize_steering_angle( currentStearingAngle,steering_angle,numOFLine)
        if steering_angle > 0 and steering_angle < 90:
            steeringRatio = -0.1
        elif steering_angle > -90 and steering_angle < 0:
            steeringRatio = 0.1
        else : steering = 0

        throttleRatio=1

    return steeringRatio,throttleRatio

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
    cv2.imshow("Live", perception(img)[0])
    steeringRatio , throttleRatio =control(img,perception(img)[1])
    cv2.waitKey(1)

    simulator.set_car_steering(steeringRatio * simulator.max_steer_angle / 1.7)
    simulator.set_car_velocity(throttleRatio * 25)


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


#             throttle_output = 0
#             brake_output    = 0
#
#             # PID Controller
#             delta_t = t - self.vars.t_last   # delta t
#             e_current = v_desired - v           # current error
#
# 			# Proportional part
#             Proportional = kp * e_current
#
#             # integral part
#             self.vars.E = self.vars.E + e_current * delta_t
#             integral = ki * self.vars.E
#
# 			# derivate part
#             if delta_t == 0:
#                 derivate = 0
#             else:
#                 derivate = kd * ((e_current - self.vars.e_previous)/delta_t)
#
#             u = Proportional + integral + derivate    # u : input signal
#
#             if u >= 0:
#                 throttle_output = u
#                 brake_output    = 0
#             elif u < 0:
#                 throttle_output = 0
#                 brake_output    = -u
#
#
#             ######################################################
#             ######################################################
#             # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
#             ######################################################
#             ######################################################
#             """
#                 Implement a lateral controller here. Remember that you can
#                 access the persistent variables declared above here. For
#                 example, can treat self.vars.v_previous like a "global variable".
#             """
#
#             # Change the steer output with the lateral controller.
#             steer_output    = 0
#
# 			# Use stanley controller for lateral control
#
#             # 0. spectify stanley parameters
#             k = 0.5
#
#             # 1. calculate heading error
#             yaw_path = np.arctan2(waypoints[-1][1]-waypoints[0][1], waypoints[-1][0]-waypoints[0][0])
#             yaw_diff = yaw_path - yaw
#             if yaw_diff > np.pi:
#                 yaw_diff -= 2 * np.pi
#             if yaw_diff < - np.pi:
#                 yaw_diff += 2 * np.pi
#
#             # 2. calculate crosstrack error
#
#             # Trajectory line    ax+by+c=0
#             # the equation of the line => y=slope * x + y_intercept
#             slope = (waypoints[-1][1]-waypoints[0][1])/ (waypoints[-1][0]-waypoints[0][0])
#             a = -slope
#             b = 1.0
#             c = (slope*waypoints[0][0]) - waypoints[0][1]
#
#             crosstrack_error = (a*x + b*y + c) / np.sqrt(a**2 + b**2)
#
#             yaw_cross_track = np.arctan2(y-waypoints[0][1], x-waypoints[0][0])
#             yaw_path2ct = yaw_path - yaw_cross_track
#             if yaw_path2ct > np.pi:
#                 yaw_path2ct -= 2 * np.pi
#             if yaw_path2ct < - np.pi:
#                 yaw_path2ct += 2 * np.pi
#             if yaw_path2ct > 0:
#                 crosstrack_error = abs(crosstrack_error)
#             else:
#                 crosstrack_error = - abs(crosstrack_error)
#
#             yaw_diff_crosstrack = np.arctan(k * crosstrack_error / (v))
#
#             print(crosstrack_error, yaw_diff, yaw_diff_crosstrack)
#
#             # 3. control low
#             steer_expect = yaw_diff + yaw_diff_crosstrack
#             if steer_expect > np.pi:
#                 steer_expect -= 2 * np.pi
#             if steer_expect < - np.pi:
#                 steer_expect += 2 * np.pi
#             steer_expect = min(1.22, steer_expect)
#             steer_expect = max(-1.22, steer_expect)
#
#             # 4. update
#             steer_output = steer_expect