import numpy as np

from config import *
from gradient_solver import *
from tsp import tsp
from controller import *

class DifferentialDriveRobot:
    def __init__(self, x, y, theta, vmax, sr):

        # robot position state
        self.x = x
        self.y = y
        self.z = 0
        # current angular state
        self.theta = theta

        self.vel = [0, 0, 0]   # current velocity [speed, omega, 0]
        
        # robot parameters
        self.vmax = vmax
        self.sensing_range = sr

        # avalable tasks
        # information in task: x, y, z, priority value, probability value
        self.task = np.empty((0, 4))
        self.finished_task = np.empty((0, 4))
        self.missed_task = np.empty((0, 4))

        # PID parameters
        self.linear_integral = 0
        self.angular_integral = 0
        self.previous_linear_error = 0
        self.previous_angular_error = 0

        self.at_goal = False

    def update_position(self, v, omega, dt, env):

        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

        # update infor about velocity
        self.vel = [v, omega, 0]

        if not self.at_goal:
            self.sense_new_task(env)

    def get_position(self):
        return [self.x, self.y, 0]
        
    def add_task(self, new_task):
        new_task = np.array(new_task).reshape(1, 4)
        self.task = np.vstack([self.task, new_task])
        
    def sense_new_task(self, env):
        # based on robot.x, robot.y and robot.theta, gen a new task

        """PARAMETER: modify the number bellow to decide density of the tasks"""
        if(np.random.rand()<0.5):
            """
            The task position is randomly generated
            within the robot's sensing range and within
            a certain angle from its current heading.

            Here each step just add 1 or 0 new task. We can add more
            """
            # task = [x, y, z]
            # z = np.random.uniform(env.zmax-20, env.zmax)
            z = env.zmax

            dis = np.random.rand()*self.sensing_range
            sen_angle = np.random.uniform(-np.pi/2,np.pi/2)

            x = self.x + dis * np.cos(self.theta+sen_angle)
            y = self.y + dis * np.sin(self.theta+sen_angle)

            task = np.array([x, y, z, 0])

            # print('Add task at location ',x,y,z)
            self.add_task(task)   

    def calculate_priority(self, v=None):
        p = np.array([self.x, self.y, self.z])

        vel = self.vel[0]

        if vel <= 0:
            vel = 0.0001
        
        vx = vel * np.cos(self.theta)
        vy = vel * np.sin(self.theta)

        v = np.array([vx, vy, 0])

        if len(self.task)==0:
            return

        points = []
        for task in self.task:
            points.append([task[0], task[1], task[2]])

        v_magnitude = np.linalg.norm(v)
        u_opp = -v / v_magnitude
        
        # Calculate projections onto the opposite direction vector
        projections = [(point, np.dot(np.array(point) - p, u_opp)) for point in points]
        
        # Calculate priority for each point
        priorities = []
        for point, proj in projections:
            dist = np.linalg.norm(np.array(point) - p)
            priority = proj * dist  # You can adjust this formula as needed
            priorities.append((point, priority))
        
        points_with_priority = [(point[0], point[1], point[2], priority) for (point, priority) in priorities]
        points_with_priority = np.array(points_with_priority)
        
        self.task = points_with_priority

    def plan_for_uav(self, uav_vel, uav_battery):

        if self.task.size == 0:
            return []  # No tasks to process, return an empty list
        
        traversable_time = uav_battery
        route = [[self.x, self.y, self.z]]
        pos_node = [0]                                   # save position of node in route
        
        current_node = np.array(route[-1])

        probot = np.array(self.get_position())    # position of robot when drone reach the last node

        vel = self.vel[0]

        if vel <= 0:
            vel = 0.0001
        
        vx = vel * np.cos(self.theta)
        vy = vel * np.sin(self.theta)

        vrobot = np.array([vx, vy, 0])

        ttime = 0                                 # travel time of the route

        avai_tasks = self.task

        # print('[LOG] planning for drone: number of task set', len(self.task), 'traversable time', traversable_time)
        """Plan a route for drone"""
        while traversable_time > 0 and len(avai_tasks) > 0:
            probot += np.array(vrobot)*ttime    # update position of robot when drone reach the last node

            next_node, pos = find_best_node(current_node, traversable_time, avai_tasks, probot, vrobot, uav_vel)

            if next_node is not None and pos is not None:
                travel_dis = np.linalg.norm(current_node - next_node)
                ttravel = travel_dis/uav_vel

                ttime += ttravel
                route.append(next_node)
                pos_node.append(pos)

                traversable_time -= ttravel
                current_node = route[-1]

                avai_tasks = np.delete(avai_tasks,pos,axis=0)
            else:
                break
        
        """ decide whether publish the route for drone or just wait for more tasks """
        cummdis = 0     # cummulative distance of the route but different with route length
        for i in range(1,len(route)):   # skip the first node cause it is the robot position
            cummdis += np.sign(self.task[pos_node[1]][3]) * np.linalg.norm(probot-route[i])
            # cummdis += np.linalg.norm(probot-route[i])
        
        if cummdis < (uav_max_time/(2*(len(route)-1)+1)) and not self.at_goal:
            return []
        else:
            # if the route is large enough, publish it by apply tsp optimize first
            for i in range(1,len(pos_node)):
                i_pos = pos_node[i]
                self.finished_task = np.vstack([self.finished_task, self.task[i_pos]])
                self.task = np.delete(self.task, i_pos, axis=0)

            route, new_cost = tsp(route, probot)

        return route
    
    def update_v_by_priority(self, v):
        if self.at_goal or len(self.task) == 0:
            return 0
        
        neg = sum(1 for task in self.task if task[3] < 0)
        pos = sum(1 for task in self.task if task[3] > 0)

        A = (pos - neg) / len(self.task)
        vnew = v - v * A
        
        # Ensure vnew stays within [robot_vmin, robot_vmax]
        vnew = max(min(vnew, robot_vmax), robot_vmin)
        
        return vnew
    
    def calculate_control(self, current_goal_index, goals):

        current_goal = goals[current_goal_index]
        
        v, omega = PID_controller(self,current_goal)

        if self.at_goal and current_goal_index >= len(goals)-1:
            return 0, 0, current_goal_index
        else:
            if self.at_goal:
                current_goal_index += 1
                self.at_goal = False
        return v, omega, current_goal_index

def find_best_node(current_node, traversable_time, task_set, robot_position, robot_vel, uav_vel):

    """ argmax        probabilites(q)
        subject to    distance(current_node,q)+distance(q,new_robot) < traversable_len  """
    
    # Note that robot_vel here has to be [vx, vy, vz]

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
