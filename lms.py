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


def laneDetection(img):
    h, w, c = img.shape
    ##thresholding
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    red_mask = cv2.inRange(hsv_frame, lower_red, upper_red)
    masked = cv2.bitwise_and(hsv_frame, img, mask=red_mask)
    edges = cv2.Canny(red_mask, 75, 150)
    cv2.imshow("Thresholding",masked)
    cv2.imshow("edges", edges)
    cv2.imshow("red_mask", red_mask)
    cv2.imshow("hsv_frame", hsv_frame)


    # # region of interest
    polygon = np.array([[(0,h),(640,h),(640,360),(350,288),(0,350)]])
    roimask = np.zeros_like(edges)
    cv2.fillPoly(roimask, polygon, 255)
    masked_image = cv2.bitwise_and(edges,roimask)
    # cv2.imshow("Mask",roimask)
    #cv2.imshow("mskdimg",masked_image)
    # plt.imshow(img)
    # plt.show()
    #cv2.imshow("area of interest", masked_image)

    #bird eye view, warping
    pts1 = np.float32([(100, 280), (580, 280), (0, h), (w, h)])
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    warpedimg = cv2.warpPerspective(masked_image, matrix, (w, h))
    #cv2.imshow("Warp", warpedimg)

    rho = 2
    theta = np.pi/180
    threshold = 40
    min_line_len = 150
    max_line_gap = 60
    line_image = np.zeros_like(img)
    lines = cv2.HoughLinesP(warpedimg, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    points = []
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                points.append(((x1 + 0.0, y1 + 0.0), (x2 + 0.0, y2 + 0.0)))
                cv2.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 5)
    #cv2.imshow("line image", line_image)
    #combinedImg = cv2.addWeighted(warpedimg, 0.8, line_image, 1, 1)  ##comibining the lines with lane image
    #cv2.imshow("combined",combinedImg)
    
    # # drawing the lane area
    lane_image = np.zeros_like(img)
    left_lane = []
    right_lane = []
    for line in lines:
        x1, y1, x2, y2 = line[0]
        slope = (y2 - y1) / (x2 - x1)
        if slope < 0:
            left_lane.append((x1, y1))
            left_lane.append((x2, y2))
        else:
            right_lane.append((x1, y1))
            right_lane.append((x2, y2))

    if len(left_lane) > 0 and len(right_lane) > 0:
        left_lane = np.array(left_lane)
        right_lane = np.array(right_lane)
        left_fit = np.polyfit(left_lane[:, 0], left_lane[:, 1], 1)
        right_fit = np.polyfit(right_lane[:, 0], right_lane[:, 1], 1)
        left_line = np.poly1d(left_fit)
        right_line = np.poly1d(right_fit)
        print(type(left_line))
        print(type(right_line))
        y1 = img.shape[0]
        y2 = int(y1 / 2)
        left_x1 = int((y1 - left_fit[1]) / left_fit[0])
        left_x2 = int((y2 - left_fit[1]) / left_fit[0])
        right_x1 = int((y1 - right_fit[1]) / right_fit[0])
        right_x2 = int((y2 - right_fit[1]) / right_fit[0])
        pts = np.array([[left_x1, y1], [left_x2, y2], [right_x2, y2], [right_x1, y1]])
        cv2.fillPoly(lane_image, [pts], (255, 255, 255)) 
       # cv2.imshow('lane image',lane_image)
        
    lane = lane_image
    return lane

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

    lane=laneDetection(img)
    
    # Control the car using keyboard
    if(len(lane)>300):
        throttle=0.2
    else:
        throttle=0

    if(len(lane)<200):
        steering=-0.2        
    else:
        steering=0   
    throttle=0
    steering=0    

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
