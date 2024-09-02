import numpy as np

# Define the environment
X_MIN = 0; X_MAX = 1000
Y_MIN = 0; Y_MAX = 1000
Z_MIN = 0; Z_MAX = 300

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

Kp_linear = 0.8             # Proportional gain for linear velocity
Kp_angular = 0.8            # Proportional gain for angular velocity
Ki_linear = 0
Ki_angular = 0
Kd_linear = 0.08
Kd_angular = 0.08

dt = 1                      # Time step

max_steps = 400             # Maximum number of simulation steps

robot_radius = 0.2          # Radius of the robot circle
robot_sensing_range = 20    # 
robot_vmax = 8              # average speed of GV, to set as normal speed (m/s)
robot_vmin = 3              # minimum bound for GV speed
robot_acceleration = 2      # 

uav_max_time = 30*60        # maximum traveling time of each uav (second)
# uav_min_range = 10        # minimum remaining battery that uav can consider taking new route
uav_avg_vel = 15            # uav average velocity (m/s)

epsilon = 3                 # allowed distance of controller (m)

num_drones = 3              # number of drone in simulation