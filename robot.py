import numpy as np

from config import *
from gradient_solver import *
# from ORTools import *

class DifferentialDriveRobot:
    def __init__(self, x, y, theta, vmax, sr):

        # robot state
        self.x = x
        self.y = y
        self.z = 0
        self.theta = theta

        self.vel = [0, 0, 0]   # current velocity
        
        # robot parameters
        self.vmax = vmax
        self.sensing_range = sr

        # avalable tasks
        # information in task: x, y, z, priority value, probability value
        self.task = np.empty((0, 5))
        self.finished_task = np.empty((0, 5))

        self.at_goal = False

    def update_position(self, v, omega, dt, env):
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

        # update infor about velocity
        self.vel = [v, omega, 0]

        if not self.at_goal:
            self.sense_new_task(env)

    def get_position(self):
        return [self.x, self.y, 0]
    
    def update_vel(self, new_vel):
        self.vel = new_vel
        
    def add_task(self, new_task):
        # if len(self.task) == 1 and np.all(self.task == 0):
        #     self.task = new_task
        # else:
        new_task = np.array(new_task).reshape(1, 5)
        self.task = np.vstack([self.task, new_task])
        
    def sense_new_task(self, env):
        # based on robot.x, robot.y and robot.theta, gen a new task

        """PARAMETER: modify the number bellow to decide density of the tasks"""
        if(np.random.rand()<0.7):
            """
            The task position is randomly generated
            within the robot's sensing range and within
            a certain angle from its current heading.

            Here each step just add 1 or 0 new task. We can add more
            """
            # task = [x, y, z]
            z = np.random.uniform(env.zmax-20, env.zmax)

            dis = np.random.rand()*self.sensing_range
            sen_angle = np.random.uniform(-np.pi/2,np.pi/2)

            x = self.x + dis * np.cos(self.theta+sen_angle)
            y = self.y + dis * np.sin(self.theta+sen_angle)

            task = np.array([x, y, z, 0, 0])

            # print('Add task at location ',x,y,z)
            self.add_task(task)   

    def calculate_priority(self, v=None):
        p = np.array([self.x, self.y, self.z])

        if v == None:
            v = np.array(self.vel)

        if len(self.task)==0:
            return

        points = []
        for task in self.task:
            points.append([task[0], task[1], task[2]])
        
        # Calculate unit vector in the opposite direction of velocity
        v_magnitude = np.linalg.norm(v)
        if v_magnitude == 0:
            raise ValueError("Velocity magnitude cannot be zero.")
        u_opp = -v / v_magnitude
        
        # Calculate projections onto the opposite direction vector
        projections = [(point, np.dot(np.array(point) - p, u_opp)) for point in points]
        
        # Calculate priority for each point
        priorities = []
        for point, proj in projections:
            dist = np.linalg.norm(np.array(point) - p)
            priority = proj * dist  # You can adjust this formula as needed
            priorities.append((point, priority))
        
        # Extract the priority values
        priority_values = np.array([priority for _, priority in priorities])
        
        # Step 1: Adjust priority values to make them positive
        min_priority = np.min(priority_values)
        adjustment = abs(min_priority) + 1  # Add 1 to ensure all values become positive
        
        adjusted_priorities = priority_values + adjustment
        
        # Step 2: Normalize the adjusted priority values
        total = np.sum(adjusted_priorities)
        
        # Step 3: Calculate probabilities
        probabilities = adjusted_priorities / total
        
        # Combine points with their probabilities
        points_with_probabilities = [(point[0], point[1], point[2], priority, prob) for (point, priority), prob in zip(priorities, probabilities)]
        points_with_probabilities = np.array(points_with_probabilities)
        
        self.task = points_with_probabilities

        # print("Updated value for task set", self.task.shape)

    def plan_for_uav(self, uav_vel, uav_battery):

        if self.task.size == 0:
            return []  # No tasks to process, return an empty list
        
        traversable_time = uav_battery
        route = []
        # current_node = route[-1]
        current_node = np.array([self.x, self.y, self.z])

        probot = np.array(self.get_position())    # position of robot when drone reach the last node
        vrobot = self.vel
        ttime = 0                                 # travel time of the route

        # print('[LOG] planning for drone: number of task set', len(self.task), 'traversable time', traversable_time)
 
        while traversable_time > 0 and len(self.task) > 0:
            probot += np.array(vrobot)*ttime    # update position of robot when drone reach the last node

            next_node, pos = find_best_node(current_node, traversable_time, self.task, probot, self.vel, uav_vel)

            if next_node is not None and pos is not None:
                travel_dis = np.linalg.norm(current_node - next_node)
                ttravel = travel_dis/uav_vel

                ttime += ttravel
                route.append(next_node)

                traversable_time -= ttravel
                current_node = route[-1]

                self.finished_task = np.vstack([self.finished_task, self.task[pos]])
                self.task = np.delete(self.task, pos, axis=0)
            else:
                break

        return route
    
    def update_v_by_priority(self, v):
        neg = sum(1 for i in range(len(self.task)) if self.task[i][3] < 0)
        pos = sum(1 for i in range(len(self.task)) if self.task[i][3] > 0)

        A = (pos-neg)/(len(self.task))

        vnew = v-v*A
        return vnew if np.linalg.norm(vnew)>robot_vmin else v

def calculate_control(robot, goal, Kp_linear, Kp_angular):
    # Calculate the error in position
    dx = goal[0] - robot.x
    dy = goal[1] - robot.y
    distance_error = np.sqrt(dx**2 + dy**2)
    
    # Calculate the desired orientation
    desired_theta = np.arctan2(dy, dx)
    
    # Calculate the error in orientation
    orientation_error = desired_theta - robot.theta
    
    # Normalize the orientation error to be within -pi to pi
    orientation_error = np.arctan2(np.sin(orientation_error), np.cos(orientation_error))
    
    # Proportional control for linear and angular velocities
    v = Kp_linear * distance_error
    omega = Kp_angular * orientation_error

    if v>robot.vmax:
        v = robot.vmax
    
    return v, omega

def find_best_node(current_node, traversable_time, task_set, robot_position, robot_vel, uav_vel):

    # argmax        probabilites(q)
    # subject to    distance(current_node,q)+distance(q,new_robot) < traversable_len

    # first find the new_robot position for each point q
    t_total = np.zeros(len(task_set))

    for i in range(len(task_set)):
        q = [task_set[i][0],task_set[i][1],task_set[i][2]]
        l1 = np.linalg.norm(q - current_node)
        t1 = l1/uav_vel

        t2 = optimize2_gradient_descent(q, l1, t1, robot_position, robot_vel, uav_vel, traversable_time)

        if t2 is None:
            # return None, None
            t_total[i] = np.inf
        else:
            t_total[i] = t1+t2

    # after that, we can apply optimization solver to find best q

    q_, position = optimizer1(task_set, current_node, robot_position, robot_vel, t_total, traversable_time)

    if q_ is None:
        return None, None

    q_ = np.array(q_)

    return q_, position

    # just need to return the array of position [x,y,z]


# Example usage:
# robot_position = (1, 1)
# velocity = (1, 0)
# points = [(4, 5), (2, 3), (0, 0), (-3, 1), (-2, 2)]

# result = calculate_priority(robot_position, velocity, points)
# print('points with probabilities',result)