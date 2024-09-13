import numpy as np
from forload import load_txt

# Define the environment
# X_MIN = 0; X_MAX = 1000
# Y_MIN = 0; Y_MAX = 1000
Z_MIN = 0; Z_MAX = 300

# Parameters
# start = (0, 0, 0)     # Starting position (x, y, theta)

Kp_linear = 0.8             # Proportional gain for linear velocity
Kp_angular = 0.8            # Proportional gain for angular velocity
Ki_linear = 0
Ki_angular = 0
Kd_linear = 0.08
Kd_angular = 0.08

dt = 1                      # Time step

max_steps = 3000             # Maximum number of simulation steps

robot_radius = 0.2          # Radius of the robot circle
robot_sensing_range = 20    # 
robot_vmax = 8              # average speed of GV, to set as normal speed (m/s)
robot_vmin = 3              # minimum bound for GV speed
robot_acceleration = 2      # 

uav_max_time = 15*60          # maximum traveling time of each uav (second)
uav_avg_vel = 12            # uav average velocity (m/s)
"""Means that UAV can travel around uav_max_time*uav_avg_vel meters. Change that in case of small scenario"""

epsilon = 30                 # allowed distance of controller (m)

num_drones = 3              # number of drone in simulation

save_file = False

filename = 'cir1.txt'

class Environment:
    def __init__(self, filename):
        # environment boundary
        # self.xmax = X_MAX
        # self.xmin = X_MIN
        # self.ymax = Y_MAX
        # self.ymin = Y_MIN
        self.zmax = Z_MAX
        self.zmin = Z_MIN

        self.load_file = False

        # search 
        if filename:
            self.load_file = True
            self.START,self.GOAL,self.sensing_range,self.tasks, self.uav_max_time = load_txt(filename)
        else:
            self.START = [0,0,0]
            self.GOAL = [5000,5000]
            self.tasks = []
            self.sensing_range = robot_sensing_range
