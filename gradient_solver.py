import numpy as np
from config import *

def optimizer1(task_set, current_node, robot_position, robot_vel, t_total, traversable_time):
    # pass
    def distance(a, b):
        return np.linalg.norm(np.array(a)-np.array(b))

    def check_feasibility(enode, cnode, pr, vr, t_total, traversable_time):
        current_node = np.array(enode)
        candidate_node = np.array(cnode)
        robot_position = np.array(pr)
        robot_vel = np.array(vr)
        
        if np.isinf(t_total):
            return False
        # Calculate new robot position after moving for t_total time
        new_robot_position = robot_position + robot_vel * t_total
        
        # Check feasibility
        return (distance(current_node, candidate_node) + distance(candidate_node, new_robot_position) <= traversable_time*uav_avg_vel)

    possibility = 0
    pos = None

    for i in range(len(task_set)):
        if task_set[i][4] > possibility and check_feasibility(current_node,np.array(task_set[i][:3]),robot_position,robot_vel,t_total[i],traversable_time):
            chosed_task = task_set[i][:3]
            pos = i
    if pos is None:
        return [], None
    return chosed_task, pos

def optimize2_gradient_descent(q, l1, t1, robot_position, robot_vel, uav_vel, traversable_time, alpha=0.01, epsilon=1e-6, max_iterations=10000):
    # Initial guess for t2
    t2 = traversable_time

    # Precompute constant parts of the constraint
    robot_pos = np.array(robot_position)
    robot_velocity = np.array(robot_vel)
    q_point = np.array(q)
    fixed_part = robot_pos + t1 * robot_velocity

    # Gradient of the objective function
    def gradient(t2):
        return uav_vel

    # Constraint function
    # def constraint(t2):
    #     new_position = fixed_part + t2 * robot_velocity
    #     return np.linalg.norm(new_position - q_point) + l1 - traversable_len
    
    def constraint(t2):
        new_position = fixed_part + t2 * robot_velocity
        return np.linalg.norm(new_position - q_point) + l1 - (traversable_time)*uav_avg_vel*0.8

    # Gradient descent loop
    for _ in range(max_iterations):
        grad = gradient(t2)
        
        # Update t2
        t2_new = t2 - alpha * grad

        if t2_new <= 0:
            break
        
        # Check if the new t2 satisfies the constraint
        t2 = t2_new
        if constraint(t2_new) < 0:
            # print('[LOG]: found t2', t2)
            break            
        
        # Check for convergence
        if abs(grad) < epsilon:
            break

    if constraint(t2) > 0:
        # print('[LOG] WARNING OPTIMIZER2: The problem does not have an optimal solution.')
        return None
    else:
        # print('[LOG]: t2 =', t2)
        return t2