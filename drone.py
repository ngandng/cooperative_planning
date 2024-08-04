import numpy as np
from robot import DifferentialDriveRobot
from config import *

from enum import Enum

class DroneState(Enum):
    WAITING = 1
    MOVING = 2
    COMEBACK = 3
    CHARGING = 4

class Drone:
    def __init__(self, id, init_pos, average_vel, battery_limit):
        self.index = id

        # drone state
        self.position = init_pos
        self.vel = average_vel
        self.battery = battery_limit
        self.state = DroneState.WAITING

        # drone plan
        self.route = []

        # operating states
        self.current_target = None

    def check_battery(self):
        return self.battery

    def set_route(self, route):
        self.route = route

    def has_tasks(self):
        return bool(self.route)

    def move_to_pos(self, target):
        direction = np.array(target) - np.array(self.position)
        distance = np.linalg.norm(direction)
        if distance > 0:
            direction = direction / distance
            move_vec = direction * self.vel
            self.position = self.position + move_vec
            self.battery -= np.linalg.norm(move_vec)  # Decrease battery with movement

    def move(self):
        if not self.current_target:
            self.current_target = self.route[0]

            del self.route[0]

        # check whether drone reach the current goal
        if np.linalg.norm(self.position-self.current_target) < epsilon:
            if len(self.route)>=1:
                self.current_target = self.route[0]
                del self.route[0]
            else:
                self.current_target = None
                self.state = DroneState.COMEBACK

        # update position
        if self.current_target:
            self.move_to_pos(self.current_target)

# # Example usage
# start = (0, 0, np.pi/4)     # Starting position (x, y, theta)
# robot_radius = 0.2          # Radius of the robot circle
# robot_sensing_range = 20    # 
# robot_vmax = 5

# uav = Drone(init_pos=[0, 0, 0], average_vel=0.1, battery_limit=20)
# robot = DifferentialDriveRobot(start[0], start[1], start[2], vmax=robot_vmax, sr=robot_sensing_range)
