import numpy as np

# Define the environment
X_MIN = 0; X_MAX = 100
Y_MIN = 0; Y_MAX = 100
Z_MIN = 0; Z_MAX = 30

class Environment:
    def __init__(self):
        self.xmax = X_MAX
        self.xmin = X_MIN
        self.ymax = Y_MAX
        self.ymin = Y_MIN
        self.zmax = Z_MAX
        self.zmin = Z_MIN

# Parameters
start = (0, 0, 0)     # Starting position (x, y, theta)

Kp_linear = 1.0             # Proportional gain for linear velocity
Kp_angular = 1.0            # Proportional gain for angular velocity

dt = 1                      # Time step

max_steps = 400              # Maximum number of simulation steps

robot_radius = 0.2          # Radius of the robot circle
robot_sensing_range = 20     # 
robot_vmax = 5
robot_vmin = 2

uav_max_time = 15          # maximum traveling distance of each uav
# uav_min_range = 10           # minimum remaining battery that uav can consider taking new route
uav_avg_vel = 15            # uav average velocity

epsilon = 0.5              # allowed distance of controller

num_drones = 3