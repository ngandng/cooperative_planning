import numpy as np
from robot import DifferentialDriveRobot
from config import *

from enum import Enum

class DroneState(Enum):
    WAITING = 1
    MOVING = 2
    COMEBACK = 3
    CHARGING = 4

    OUT_OF_CONTROL = 5

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

    def charge(self, battery_limit):
        self.battery = battery_limit

    def has_tasks(self):
        return bool(self.route)

    def move_to_pos(self, target):
        direction = np.array(target) - np.array(self.position)
        distance = np.linalg.norm(direction)

        velocity = self.vel

        if distance > epsilon:
            direction = direction / distance
            while distance < velocity:
                velocity = velocity*0.9

            move_vec = direction * velocity
            
            self.position = self.position + move_vec
            self.battery -= np.linalg.norm(move_vec)/uav_avg_vel  # Decrease battery with movement
            # self.battery -= 1

        direction = np.array(target) - np.array(self.position)
        distance = np.linalg.norm(direction)
        if distance > epsilon*2 and self.battery <= 0:
                print('\n [WARNING] OUT_OF_CONTROL: drone position [%s,%s,%s], distance to the GV %s \n'%(self.position[0],self.position[1],self.position[2], distance))
                self.state = DroneState.OUT_OF_CONTROL


    def move(self):
        if self.current_target is None:
            self.current_target = self.route[0]

            del self.route[0]

        # check whether drone reach the current goal
        dis2tar = np.linalg.norm(self.position-self.current_target)
        # print('Drone ', self.index, 'checking distance to target ', dis2tar)
        if dis2tar < epsilon:
            if len(self.route)>=1:
                self.current_target = self.route[0]
                del self.route[0]
            else:
                self.current_target = None
                print('[annouce] drone', self.index, 'finished the mission and now comming back')
                self.state = DroneState.COMEBACK

        # update position
        if self.current_target is not None:
            self.move_to_pos(self.current_target)
        
        return self.current_target

# # Example usage
# start = (0, 0, np.pi/4)     # Starting position (x, y, theta)
# robot_radius = 0.2          # Radius of the robot circle
# robot_sensing_range = 20    # 
# robot_vmax = 5

# uav = Drone(init_pos=[0, 0, 0], average_vel=0.1, battery_limit=20)
# robot = DifferentialDriveRobot(start[0], start[1], start[2], vmax=robot_vmax, sr=robot_sensing_range)
