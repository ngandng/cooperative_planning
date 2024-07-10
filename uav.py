import numpy as np
from robot import DifferentialDriveRobot

# State.py
class State:
    def run(self, uav, robot):
        raise NotImplementedError("run method not implemented")

    def next(self, uav, robot):
        raise NotImplementedError("next method not implemented")

# Statemachine.py
class StateMachine:
    def __init__(self, initial_state, uav, robot):
        self.current_state = initial_state
        self.uav = uav
        self.robot = robot
        self.current_state.run(self.uav, self.robot)

    def run_all(self, inputs):
        for input in inputs:
            print(input)
            self.current_state = self.current_state.next(self.uav, self.robot)
            self.current_state.run(self.uav, self.robot)

# UAV States
class UAVStayingInStation(State):
    def run(self, uav, robot):
        print("UAV staying in station")

    def next(self, uav, robot):
        if uav.check_battery() and uav.has_tasks():
            return UAVQueryingForRoute()
        return self

class UAVQueryingForRoute(State):
    def run(self, uav, robot):
        print("UAV querying for route")

    def next(self, uav, robot):
        route = robot.plan_for_uav()
        if route:
            uav.update_route(route)
            return UAVMovingToTask()
        return self

class UAVMovingToTask(State):
    def run(self, uav, robot):
        print("UAV moving to task")

    def next(self, uav, robot):
        if uav.has_tasks():
            current_target = uav.route[0]
            if np.linalg.norm([uav.position[0] - current_target[0], uav.position[1] - current_target[1], uav.position[2] - current_target[2]]) < 0.01:
                new_route = uav.route[1:]  # Exclude the reached target
                uav.update_route(new_route)
            uav.move_to(current_target)
        else:
            return UAVGoingBack()
        return self

class UAVGoingBack(State):
    def run(self, uav, robot):
        print("UAV going back to the station")

    def next(self, uav, robot):
        if np.linalg.norm([uav.position[0] - robot.x, uav.position[1] - robot.y, uav.position[2]]) < 0.01:
            return UAVStayingInStation()
        else:
            uav.move_to(robot.position)
        return self

# UAV class
class UAV:
    def __init__(self, init_pos, average_vel, battery_limit):
        self.position = init_pos
        self.vel = average_vel
        self.battery = battery_limit
        self.route = []

    def check_battery(self):
        return self.battery >= 5

    def update_route(self, route):
        self.route = route

    def has_tasks(self):
        return bool(self.route)

    def move_to(self, target):
        direction = np.array(target) - np.array(self.position)
        distance = np.linalg.norm(direction)
        if distance > 0:
            direction = direction / distance
            move_vec = direction * self.vel
            self.position = self.position
            self.battery -= np.linalg.norm(move_vec)  # Decrease battery with movement

# Example usage
start = (0, 0, np.pi/4)     # Starting position (x, y, theta)
robot_radius = 0.2          # Radius of the robot circle
robot_sensing_range = 20     # 
robot_vmax = 5

uav = UAV(init_pos=[0, 0, 0], average_vel=0.1, battery_limit=20)
robot = DifferentialDriveRobot(start[0], start[1], start[2], vmax=robot_vmax, sr=robot_sensing_range)
initial_state = UAVStayingInStation()
state_machine = StateMachine(initial_state, uav, robot)

# Run the state machine with some inputs
inputs = [None] * 10  # Example input list
state_machine.run_all(inputs)
