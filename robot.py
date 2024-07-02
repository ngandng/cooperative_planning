import numpy as np

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

class DifferentialDriveRobot:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def update_position(self, v, omega, dt):
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt
        self.theta += omega * dt

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
    
    return v, omega

def sense_new_task(robot):
    # based on robot.x, robot.y and robot.theta, gen a new task
    # task = [x, y, z]
    z = np.random.uniform(Z_MAX-20, Z_MAX)
    dis = np.random
    x += dis * np.cos(robot.theta)
    y += dis * np.sin(robot.theta)
    task = [robot.x, robot.y, z]
    return task

def find_opposite_and_furthest_points(robot_position, velocity, points):
    p = np.array(robot_position)
    v = np.array(velocity)
    
    # Calculate unit vector in the opposite direction of velocity
    v_magnitude = np.linalg.norm(v)
    u_opp = -v / v_magnitude
    
    # Calculate projections onto the opposite direction vector
    projections = [(point, np.dot(np.array(point) - p, u_opp)) for point in points]
    
    # Find the maximum projection
    max_proj = max(projections, key=lambda x: x[1])[1]
    
    # Filter points that have this maximum projection
    max_proj_points = [point for point, proj in projections if proj == max_proj]
    
    # Calculate distances from the robot's position to these points
    distances = [(point, np.linalg.norm(np.array(point) - p)) for point in max_proj_points]
    
    # Find the maximum distance
    max_distance = max(distances, key=lambda x: x[1])[1]
    
    # Filter points that have this maximum distance
    max_dist_points = [point for point, dist in distances if dist == max_distance]
    
    return max_dist_points

# Example usage:
robot_position = (1, 1)
velocity = (1, 0)
points = [(4, 5), (2, 3), (0, 0), (-3, 1), (-2, 2)]

result = find_opposite_and_furthest_points(robot_position, velocity, points)
print(result)