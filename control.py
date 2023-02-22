import math
import time
import globalVariable


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

    if globalVariable.leftLineOnlyExit :
        steeringRatio = -1
        throttleRatio=0.2
        numOFLine=1

    elif globalVariable.rightLineOnlyExit:
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
