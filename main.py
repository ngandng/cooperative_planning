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

    for _ in range(max_steps):
        v, omega = calculate_control(robot, goal, Kp_linear, Kp_angular)

        # update new position and sense for new tasks
        robot.update_position(v, omega, dt, env)
        
        trajectory.append((robot.x, robot.y))

        task_list.append(robot.task)
        
        # Stop if the robot is close enough to the goal
        if np.linalg.norm([robot.x - goal[0], robot.y - goal[1]]) < 0.01:
            break
    
    return trajectory, task_list

# Simulate the robot
trajectory, task_list = simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps)

print('taks_list.len ', len(task_list))
for i in range(len(task_list)):
    print(task_list[i].shape)
# Plotting the trajectory as an animation
trajectory = np.array(trajectory)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim((min(trajectory[:, 0]) - 1, max(trajectory[:, 0]) + 1))
ax.set_ylim((min(trajectory[:, 1]) - 1, max(trajectory[:, 1]) + 1))
ax.set_zlim(0, 30)
line, = ax.plot([], [], 'b-', label='Robot Trajectory')
start_marker, = ax.plot([], [], 'bo', label='Start')
goal_marker, = ax.plot([], [], 'ro', label='Goal')
robot_circle, = ax.plot([], [], 'ys', label='Robot', markersize=20)
robot_direction, = ax.plot([], [], 'r-')
task_scatter, = ax.plot([], [], [], c='g', marker='o', label='Task', linestyle='', markersize=7)

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.legend(loc='upper left')
ax.set_title('Trajectory of Differential Drive Robot')
ax.grid(True)

def init():
    line.set_data([], [])
    start_marker.set_data([start[0]], [start[1]])
    goal_marker.set_data([goal[0]], [goal[1]])
    robot_circle.set_data([start[0]], [start[1]])
    robot_direction.set_data([], [])
    task_scatter.set_data([], [], [])

    return line, start_marker, goal_marker, robot_circle, robot_direction, task_scatter

def update_frame(frame):
    line.set_data(trajectory[:frame, 0], trajectory[:frame, 1])
    robot_circle.set_data([trajectory[frame, 0]], [trajectory[frame, 1]])

    # Calculate the direction line based on robot's theta
    direction_length = 5.0
    direction_x = trajectory[frame, 0] + direction_length * np.cos(start[2])
    direction_y = trajectory[frame, 1] + direction_length * np.sin(start[2])
    robot_direction.set_data([trajectory[frame, 0], direction_x],
                             [trajectory[frame, 1], direction_y])

    if frame < len(task_list):
        tasks = task_list[frame]
        if len(tasks) > 0:
            tx = tasks[:, 0]
            ty = tasks[:, 1]
            tz = tasks[:, 2]
            task_scatter.set_data(tx, ty, tz)
        else:
            task_scatter.set_data([], [], [])

    return line, start_marker, goal_marker, robot_circle, robot_direction, task_scatter

ani = FuncAnimation(fig, update_frame, frames=len(trajectory), init_func=init, blit=True, repeat=False)

# Save the animation as a GIF
ani.save('robot_trajectory.gif', writer=PillowWriter(fps=10))

plt.show()
