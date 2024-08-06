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

# define the robot
robot = DifferentialDriveRobot(start[0], start[1], start[2], vmax=robot_vmax, sr=robot_sensing_range)

#define the drones
drone = [Drone(i, init_pos=[0, 0, 0], average_vel=uav_avg_vel, battery_limit=uav_max_range) for i in range(num_drones)]

def simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps):
    
    trajectory = [(robot.x, robot.y)]
    drones_traj = [
    {"positions": [(0, 0)]},  # Drone 0
    {"positions": [(0, 0)]},  # Drone 1
    {"positions": [(0, 0)]}   # Drone 2
    ]

    task_list = []
    finished_task = []

    env = Environment()

    for _i in range(max_steps):
        print('[LOG] STEP', _i)
        v, omega = calculate_control(robot, goal, Kp_linear, Kp_angular)

        if _i > 1:
            robot.calculate_priority()

            if (len(robot.task)>0):
                v = robot.update_v_by_priority(v)

        # update new position and sense for new tasks
        robot.update_position(v, omega, dt, env)
            
        for _drone in drone:

            drones_traj[_drone.index]["positions"].append((_drone.position[0],_drone.position[1],_drone.position[2]))

            if _drone.state == DroneState.COMEBACK:
                print('[LOG] drone', _drone.index, 'is comming back')
                # if drone is comming back, send robot position for drone
                _drone.move_to_pos(robot.get_position())
                if np.linalg.norm(_drone.position-robot.get_position()) < epsilon:
                    _drone.state = DroneState.CHARGING

            if _drone.state == DroneState.CHARGING:
                print('[LOG] drone', _drone.index, 'is charging')
                _drone.charge(uav_max_range)

                # wait for charge until drone full of battery
                if _drone.check_battery() >= uav_min_range:
                    _drone.state = DroneState.WAITING

            # when drone is waiting
            if _i > 10:
                if _drone.state == DroneState.WAITING:
                    print('[LOG] drone', _drone.index, 'is waiting for the route')
                    robot.calculate_priority()
                    plan = robot.plan_for_uav(_drone.vel,_drone.check_battery())
                    _drone.position = robot.get_position()

                    if plan:
                        _drone.set_route(plan)
                        print("[LOG] New plan loaded on the drone", _drone.index)
                        _drone.state = DroneState.MOVING

            # when drone state is moving
            if _drone.state == DroneState.MOVING:
                current_target = _drone.move()
                if current_target is not None:
                    print('[LOG] drone', _drone.index, 'is moving to [%s, %s, %s]' % (current_target[0], current_target[1], current_target[2]))

        task_info = robot.task

        # if not task_info is None:
        #     print('LOGGING OF STEP ',_i)
        #     for i in range(len(task_info)):
        #         print('Task ', i, ':: location ', task_info[i][0],task_info[i][1],task_info[i][2],' priority value ', task_info[i][3],' prob ', task_info[i][4])
        
        trajectory.append((robot.x, robot.y))
        task_list.append(robot.task)
        finished_task.append(robot.finished_task)

        if np.linalg.norm([robot.x - goal[0], robot.y - goal[1]]) < epsilon:
            robot.at_goal = True
        
        if robot.at_goal and sum(1 for _drone in drone if _drone.state == DroneState.WAITING):
            break
    
    return trajectory, drones_traj, task_list, finished_task

# Simulate the robot
trajectory, drones_traj, task_list, finished_task = simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps)

print('[LOG] Finished the simulation, number of unassigned task', len(robot.task), 'number of assigned task', len(robot.finished_task))

# Plotting the trajectory as an animation using the TrajectoryPlotter class
plotter = TrajectoryPlotter(trajectory, drones_traj, task_list, start, goal, finished_task)
plotter.animate('robot_trajectory.gif', fps=5)
