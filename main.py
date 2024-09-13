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
# from robot_trajectory import *


env = Environment(filename)

# define the robot
robot = DifferentialDriveRobot(env.START[0],env.START[1],env.START[2],vmax=robot_vmax,sr=env.sensing_range)

#define the drones
drone = [Drone(i, init_pos=env.START, average_vel=uav_avg_vel, battery_limit=env.uav_max_time) for i in range(num_drones)]

# some metrics for evaluation
total_route = 0     # total number of route assign for drones 

def simulate_robot(dt, max_steps):
    
    # saving information for plot
    robot_state = [(robot.x, robot.y, 0, robot.theta)]
    robot_vel = []
    drones_info = [
    {"positions": [env.START], "battery": [100]},  # Drone 0
    {"positions": [env.START], "battery": [100]},  # Drone 1
    {"positions": [env.START], "battery": [100]}   # Drone 2
    ]

    task_list = []
    finished_task = []

    # working metrics
    count_for_break = 0
    current_goal_index = 0     # index of current target on list of target

    """Loop of system operation"""
    for iter in range(max_steps):
        print('[LOG] STEP', iter)
        v, omega, current_goal_index = robot.calculate_control(current_goal_index, env.GOAL)

        if iter > 1:
            robot.calculate_priority()

            if (len(robot.task)>0):
                v = robot.update_v_by_priority(v)

        # update new position and sense for new tasks
        robot.update_position(v, omega, dt, env)
            
        for _drone in drone:

            drones_info[_drone.index]["positions"].append((_drone.position[0],_drone.position[1],_drone.position[2]))
            drones_info[_drone.index]["battery"].append(_drone.battery/env.uav_max_time*100)

            if _drone.state == DroneState.COMEBACK:
                print('[LOG] drone', _drone.index, 'is comming back: position [%s,%s,%s]; battery state %s' % (_drone.position[0],_drone.position[1],_drone.position[2],_drone.battery))
                # if drone is comming back, send robot position for drone
                _drone.move_to_pos(robot.get_position())
                if np.linalg.norm(_drone.position-robot.get_position()) < epsilon:
                    _drone.state = DroneState.CHARGING

            if _drone.state == DroneState.CHARGING:
                print('[LOG] drone', _drone.index, 'is charging')
                _drone.charge(env.uav_max_time)

                # wait for charge until drone full of battery
                if _drone.battery >= env.uav_max_time:
                    _drone.state = DroneState.WAITING

            if _drone.state == DroneState.OUT_OF_CONTROL:
                # print('[LOG] IMPORTANT: Drone', _drone.index, 'is now run out of battery and crash !!!!!!!!!!!!!')
                pass

            # when drone is waiting
            if iter > 5:
                if _drone.state == DroneState.WAITING:
                    _drone.count_for_charging = 1
                    # print('[LOG] drone', _drone.index, 'is waiting for the route')
                    robot.calculate_priority()
                    plan = robot.plan_for_uav(_drone.vel,_drone.battery)
                    _drone.position = robot.get_position()

                    if len(plan) > 1:
                        _drone.set_route(plan)
                        global total_route
                        total_route += 1
                        print("[LOG] New plan loaded on the drone", _drone.index, '. Number of task', len(plan)-1)
                        _drone.state = DroneState.MOVING

                        count_for_break = 0

            # when drone state is moving
            if _drone.state == DroneState.MOVING:
                current_target = _drone.move()
                if current_target is not None:
                    print('[LOG] drone', _drone.index, 'is moving to [%s, %s, %s]' % (current_target[0], current_target[1], current_target[2]))
        
        robot_state.append((robot.x, robot.y, 0, robot.theta))
        robot_vel.append(v)
        task_list.append(robot.task)
        finished_task.append(robot.finished_task)
        
        if ((robot.at_goal and sum(1 for _drone in drone if _drone.state==DroneState.WAITING or _drone.state==DroneState.OUT_OF_CONTROL)==3)):
            count_for_break += 1
            if count_for_break > 5:
                break

    return robot_state, robot_vel, drones_info, task_list, finished_task

def main():

    # goals = np.array(circular_path())

    # Simulate the robot
    robot_state, vr, drones_info, task_list, finished_task = simulate_robot(dt, max_steps)

    print('[FINISH LOG] Finished the simulation, number of unassigned task', len(robot.task), 'number of assigned task', len(robot.finished_task), 'number of crashed drone', sum(1 for _drone in drone if _drone.state == DroneState.OUT_OF_CONTROL))
    print('Number of route for drones', total_route)
    print('[FINISH LOG] Drones state')
    for _drone in drone:
        print('Drone', _drone.index, 'state', _drone.state)

    
    print('Checking unassigned the task list:')
    for task in robot.task:
        print('Position ', task[0:3], 'priority value', task[-1])

    # Plotting the trajectory as an animation using the TrajectoryPlotter class
    plotter = TrajectoryPlotter(robot_state, drones_info, task_list, env.START, env.GOAL, finished_task)
    plotter.plot_drone_lines()
    plotter.plot_robot_vel(vr)
    # plotter.plot_batteryiternfo()
    if save_file:
        plotter.animate('robot_trajectory.gif', fps=15)

    print('Check the plotter. Number of available task ', len(robot.task), 'number of finished tasks', len(robot.finished_task))

if __name__ == "__main__":
    main()