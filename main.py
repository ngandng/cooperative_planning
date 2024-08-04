"""
Simulation:
    - Input: 
            - A planned path for robot
    - Robot: 
            - moving to follow the path
            - sense the environment around to track the task around current position
            - save the remaining task in to a set
            - compute to assign route for drone
    - Drone:
            - Each drone query it's route from drone
            - Moving to task set by the route
            - Comeback to robot to charge and get new mission
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D

from robot import *
from drone import *
from config import *
from plot import *

def simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps):
    robot = DifferentialDriveRobot(start[0], start[1], start[2], vmax=robot_vmax, sr=robot_sensing_range)

    trajectory = [(robot.x, robot.y)]
    task_list = []

    env = Environment()

    drone = [Drone(i, init_pos=[0, 0, 0], average_vel=1.0, battery_limit=uav_max_range) for i in range(num_drones)]

    for _i in range(max_steps):
        v, omega = calculate_control(robot, goal, Kp_linear, Kp_angular)

        # update new position and sense for new tasks
        robot.update_position(v, omega, dt, env)
            
        for _drone in drone:

            if _drone.state == DroneState.COMEBACK:
                # if drone is comming back, send robot position for drone
                _drone.move_to_pos(robot.get_position())
                if np.linalg.norm(_drone.position-robot.get_position()) < epsilon:
                    _drone.state = DroneState.CHARGING

            if _drone.state == DroneState.CHARGING:
                _drone.battery = uav_max_range

                # wait for charge until drone full of battery
                if _drone.check_battery == uav_max_range:
                    _drone.state = DroneState.WAITING

            # when drone is waiting
            if _drone.state == DroneState.WAITING:
                robot.calculate_priority()
                plan = robot.plan_for_uav(_drone.vel,_drone.check_battery())
                _drone.position = robot.get_position()

                if plan:
                    _drone.set_route(plan)
                    print(f"Set a new plan for the drone {_drone.index}: {plan}")
                    _drone.state = DroneState.MOVING

            # when drone state is moving
            if _drone.state == DroneState.MOVING:
                _drone.move()

        task_info = robot.task

        # if not task_info is None:
        #     print('LOGGING OF STEP ',_i)
        #     for i in range(len(task_info)):
        #         print('Task ', i, ':: location ', task_info[i][0],task_info[i][1],task_info[i][2],' priority value ', task_info[i][3],' prob ', task_info[i][4])
        
        trajectory.append((robot.x, robot.y))
        task_list.append(robot.task)
        
        if np.linalg.norm([robot.x - goal[0], robot.y - goal[1]]) < epsilon:
            break
    
    return trajectory, task_list

# Simulate the robot
trajectory, task_list = simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps)

# Plotting the trajectory as an animation using the TrajectoryPlotter class
plotter = TrajectoryPlotter(trajectory, task_list, start, goal)
plotter.animate('robot_trajectory.gif', fps=10)