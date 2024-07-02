import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import matplotlib.patches as patches

from robot import *

# Define the environment
X_MIN = 0; X_MAX = 100
Y_MIN = 0; Y_MAX = 100
Z_MIN = 0; Z_MAX = 30


# Parameters
start = (0, 0, np.pi/4)  # Starting position (x, y, theta)
goal = (95, 95)       # Goal position (x, y)
Kp_linear = 1.0     # Proportional gain for linear velocity
Kp_angular = 2.0    # Proportional gain for angular velocity
dt = 1            # Time step
max_steps = 50      # Maximum number of simulation steps
robot_radius = 0.2  # Radius of the robot circle

def simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps):
    robot = DifferentialDriveRobot(start[0], start[1], start[2])
    trajectory = [(robot.x, robot.y)]

    for _ in range(max_steps):
        v, omega = calculate_control(robot, goal, Kp_linear, Kp_angular)
        robot.update_position(v, omega, dt)
        trajectory.append((robot.x, robot.y))
        
        # Stop if the robot is close enough to the goal
        if np.linalg.norm([robot.x - goal[0], robot.y - goal[1]]) < 0.01:
            break
    
    return trajectory

# Simulate the robot
trajectory = simulate_robot(start, goal, Kp_linear, Kp_angular, dt, max_steps)

# Plotting the trajectory as an animation
trajectory = np.array(trajectory)
fig, ax = plt.subplots()
ax.set_xlim((min(trajectory[:,0])-1, max(trajectory[:,0])+1))
ax.set_ylim((min(trajectory[:,1])-1, max(trajectory[:,1])+1))
line, = ax.plot([], [], 'b-', label='Robot Trajectory')
start_marker, = ax.plot([], [], 'bo', label='Start')
goal_marker, = ax.plot([], [], 'ro', label='Goal')
robot_circle, = ax.plot([], [], 'ys', label='Robot',markersize=20)

ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.legend()
ax.set_title('Trajectory of Differential Drive Robot')
ax.grid(True)

def init():
    line.set_data([], [])
    start_marker.set_data([start[0]], [start[1]])
    goal_marker.set_data([goal[0]], [goal[1]])
    robot_circle.set_data([start[0]], [start[1]])
    return line, start_marker, goal_marker, robot_circle

def update_frame(frame):
    line.set_data(trajectory[:frame, 0], trajectory[:frame, 1])
    robot_circle.set_data([trajectory[frame, 0]], [trajectory[frame, 1]])
    return line, start_marker, goal_marker, robot_circle

ani = FuncAnimation(fig, update_frame, frames=len(trajectory), init_func=init, blit=True, repeat=False)

# Save the animation as a GIF
ani.save('robot_trajectory.gif', writer=PillowWriter(fps=10))

plt.show()
