import numpy as np
from config import *

def simple_controller(robot, current_goal):
    dx = current_goal[0] - robot.x
    dy = current_goal[1] - robot.y
    distance_error = np.sqrt(dx**2 + dy**2)

    if distance_error <= epsilon and not robot.at_goal:
        robot.at_goal = True
    
    # Calculate the desired orientation
    desired_theta = np.arctan2(dy, dx)
    
    # Calculate the error in orientation
    orientation_error = desired_theta - robot.theta
    
    # Normalize the orientation error to be within -pi to pi
    orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))

    v = distance_error

    omega = orientation_error

    # Cap the linear velocity
    if v > robot.vmax:
        v = robot.vmax
    
    return v, omega

def PID_controller(robot, current_goal):
    dx = current_goal[0] - robot.x
    dy = current_goal[1] - robot.y
    distance_error = np.sqrt(dx**2 + dy**2)

    if distance_error <= epsilon and not robot.at_goal:
        robot.at_goal = True
    
    # Calculate the desired orientation
    desired_theta = np.arctan2(dy, dx)
    
    # Calculate the error in orientation
    orientation_error = desired_theta - robot.theta
    
    # # Normalize the orientation error to be within -pi to pi
    orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))
    
    # Update the integrals
    robot.linear_integral += distance_error * dt
    robot.angular_integral += orientation_error * dt
    
    # Calculate derivatives
    linear_derivative = (distance_error - robot.previous_linear_error) / dt
    angular_derivative = (orientation_error - robot.previous_angular_error) / dt
    
    # PID control for linear and angular velocities
    v = (Kp_linear * distance_error +
            Ki_linear * robot.linear_integral +
            Kd_linear * linear_derivative)

    omega = (Kp_angular * orientation_error +
                Ki_angular * robot.angular_integral +
                Kd_angular * angular_derivative)
    
    # Update previous errors
    robot.previous_linear_error = distance_error
    robot.previous_angular_error = orientation_error
    
    # Cap the linear velocity
    if v > robot.vmax:
        v = robot.vmax
    
    return v, omega