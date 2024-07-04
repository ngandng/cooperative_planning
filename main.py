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

# Define the environment
X_MIN = 0; X_MAX = 100
Y_MIN = 0; Y_MAX = 100
Z_MIN = 0; Z_MAX = 30

# Parameters
start = (0, 0, np.pi/4)     # Starting position (x, y, theta)
goal = (200, 200)             # Goal position (x, y)
Kp_linear = 1.0             # Proportional gain for linear velocity
Kp_angular = 2.0            # Proportional gain for angular velocity
dt = 1                      # Time step
max_steps = 100              # Maximum number of simulation steps
robot_radius = 0.2          # Radius of the robot circle
robot_sensing_range = 20     # 

uav_max_range = 20          # maximum traveling distance of each uav

class Environment:
    def __init__(self):
        self.xmax = X_MAX
        self.xmin = X_MIN
        self.ymax = Y_MAX
        self.ymin = Y_MIN
        self.zmax = Z_MAX
        self.zmin = Z_MIN

def simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps):
    robot = DifferentialDriveRobot(start[0], start[1], start[2], vmax=5, sr=robot_sensing_range)

    trajectory = [(robot.x, robot.y)]
    task_list = []

    env = Environment()

    for _i in range(max_steps):
        v, omega = calculate_control(robot, goal, Kp_linear, Kp_angular)

        # update new position and sense for new tasks
        robot.update_position(v, omega, dt, env)

        task_info = calculate_priority([robot.x, robot.y, robot.z], [v, omega, 0], robot.task)

        print('LOGGING OF STEP ',_i)
        for i in range(len(task_info)):
            print('Task ', i, ':: location ', task_info[i][0], ' priority value ', task_info[i][1], ' prob ', task_info[i][2])
        
        trajectory.append((robot.x, robot.y))

        task_list.append(robot.task)
        
        # Stop if the robot is close enough to the goal
        if np.linalg.norm([robot.x - goal[0], robot.y - goal[1]]) < 0.01:
            break
    
    return trajectory, task_list

# Simulate the robot
trajectory, task_list = simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps)

# Plotting the trajectory as an animation
trajectory = np.array(trajectory)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim((min(trajectory[:, 0]) - 1, max(trajectory[:, 0]) + 1))
ax.set_ylim((min(trajectory[:, 1]) - 1, max(trajectory[:, 1]) + 1))
ax.set_zlim(0, 30)

line, = ax.plot([], [], [], 'b-', label='Robot Trajectory')
start_marker, = ax.plot([], [], [], 'bo', label='Start')
goal_marker, = ax.plot([], [], [], 'ro', label='Goal')
robot_circle, = ax.plot([], [], [], 'yo', label='Robot', markersize=20)
robot_direction, = ax.plot([], [], [], 'r-')
task_scatter = ax.scatter([], [], [], c='g', marker='o', label='Task', s=50)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend(loc='upper left')
ax.set_title('Trajectory of Differential Drive Robot')
ax.grid(True)

def init():
    line.set_data_3d([], [], [])
    start_marker.set_data_3d([start[0]], [start[1]], [0])
    goal_marker.set_data_3d([goal[0]], [goal[1]], [0])
    robot_circle.set_data_3d([start[0]], [start[1]], [0])
    robot_direction.set_data_3d([], [], [])
    task_scatter._offsets3d = ([], [], [])

    return line, start_marker, goal_marker, robot_circle, robot_direction, task_scatter

def update_frame(frame):
    line.set_data_3d(trajectory[:frame, 0], trajectory[:frame, 1], np.zeros(frame))
    robot_circle.set_data_3d([trajectory[frame, 0]], [trajectory[frame, 1]], [0])

    # Calculate the direction line based on robot's theta
    direction_length = 10.0
    direction_x = trajectory[frame, 0] + direction_length * np.cos(start[2])
    direction_y = trajectory[frame, 1] + direction_length * np.sin(start[2])
    robot_direction.set_data_3d([trajectory[frame, 0], direction_x],
                                [trajectory[frame, 1], direction_y],
                                [0, 0])

    if frame < len(task_list):
        tasks = task_list[frame]
        print('task_list[frame].shape ', tasks.shape)
        if len(tasks) > 0:
            tx = tasks[:, 0]
            ty = tasks[:, 1]
            tz = tasks[:, 2]
            task_scatter._offsets3d = (tx,ty,tz)
        else:
            task_scatter._offsets3d = ([], [], [])

    return line, start_marker, goal_marker, robot_circle, robot_direction, task_scatter

ani = FuncAnimation(fig, update_frame, frames=len(trajectory), init_func=init, blit=True, repeat=False)

# Save the animation as a GIF
ani.save('robot_trajectory.gif', writer=PillowWriter(fps=10))

plt.show()