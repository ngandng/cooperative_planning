import numpy as np
from config import *

def optimizer1(task_set, current_node, robot_position, robot_vel, t_total, traversable_time):
    
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
        return (distance(current_node, candidate_node) + distance(candidate_node, new_robot_position) < traversable_time*uav_avg_vel)

    priority = -np.inf
    pos = None

    for i in range(len(task_set)):
        if task_set[i][3] > priority:
            if check_feasibility(current_node,np.array(task_set[i][:3]),robot_position,robot_vel,t_total[i],traversable_time):
                chosed_task = task_set[i][:3]
                pos = i
            else:
                # print("I found a good task but it is out of reach")
                pass
    if pos is None:
        return [], None
    return chosed_task, pos

def optimize2_gradient_descent(q, l1, t1, robot_position, robot_vel, ttime, uav_vel, traversable_time, alpha=0.01, epsilon=1e-6, max_iterations=10000):
    # Initial guess for t2
    t2 = traversable_time

    # Precompute constant parts of the constraint
    pr = np.array(robot_position)
    vr = np.array(robot_vel)
    q = np.array(q)
    # pr_t1 = pr + t1 * vr

    # Gradient of the objective function
    def gradient(t2):
        return uav_vel

    def constraint(t2):
        z = q[2]
        d2 = np.linalg.norm([q[0]-pr[0],q[1]-pr[1]]) + np.linalg.norm((ttime+t1+t2)*robot_vmax)
        d1 = z
        max_dis = np.sqrt(d1**2 + d2**2)
        return max_dis - (ttime+t1+t2)*uav_avg_vel

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