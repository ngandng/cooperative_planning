import numpy as np

class DifferentialDriveRobot:
    def __init__(self, x, y, theta, vmax, sr):
        self.x = x
        self.y = y
        self.z = 0
        self.theta = theta

        self.vmax = vmax
        
        self.sensing_range = sr
        self.task = np.empty((1,3))

    def update_position(self, v, omega, dt, env):
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

        self.sense_new_task(env)
        
    def add_task(self, new_task):
        # if(new_task):
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

            task = np.array([x, y, z])

            # print('Add task at location ',x,y,z)
            self.add_task(task)


class Drone:
    def __init__(self,maximum_energy):
        # the maximum distance that the drone can travel due the batery constraint
        self.C = maximum_energy     

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

def calculate_priority(robot_position, velocity, points):
    p = np.array(robot_position)
    v = np.array(velocity)
    
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
    points_with_probabilities = [(point, priority, prob) for (point, priority), prob in zip(priorities, probabilities)]
    
    return points_with_probabilities

# Example usage:
robot_position = (1, 1)
velocity = (1, 0)
points = [(4, 5), (2, 3), (0, 0), (-3, 1), (-2, 2)]

result = calculate_priority(robot_position, velocity, points)
print('points with probabilities',result)