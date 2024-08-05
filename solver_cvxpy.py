import numpy as np
import cvxpy as cp

def optimize2(q, l1, t1, robot_position, robot_vel, uav_vel, traversable_len):
    # Define the optimization variable
    t2 = cp.Variable()

    # Precompute constant parts of the constraint
    robot_pos = np.array(robot_position)
    robot_velocity = np.array(robot_vel)
    q_point = np.array(q)
    fixed_part = robot_pos + t1 * robot_velocity

    # Define the constraint norm((fixed_part + t2 * robot_velocity) - q) + l1 <= traversable_len
    new_position = fixed_part + t2 * robot_velocity
    distance_squared = cp.sum_squares(new_position - q_point)
    constraint = [cp.sqrt(distance_squared) + l1 <= traversable_len, t2 >= 0]

    # Define the objective: Minimize t2 * uav_vel
    objective = cp.Minimize(t2 * uav_vel)

    # Formulate the problem
    problem = cp.Problem(objective, constraint)

    # Solve the problem
    problem.solve()

    if problem.status == cp.OPTIMAL:
        print('[LOG]: t2 =', t2.value)
        return t2.value
    else:
        print('[LOG] WARNING: The problem does not have an optimal solution.')
        return None

def optimize1(task_set, current_node, robot_position, robot_vel, t_total, traversable_len):
    num_tasks = len(task_set)

    # Variables: Binary variables to select a task
    x = cp.Variable(num_tasks, boolean=True)

    # Objective: Maximize the task value
    task_values = np.array([task[4] for task in task_set])
    objective = cp.Maximize(task_values @ x)

    # Precompute distances
    current_node = np.array(current_node)
    robot_position = np.array(robot_position)
    robot_vel = np.array(robot_vel)

    constraints = []
    for j in range(num_tasks):
        q = np.array(task_set[j][:3])
        t_total_j = t_total[j]

        # Constraint: distance(current_node, q) + distance(q, new_robot_position) <= traversable_len
        new_robot_position = robot_position + t_total_j * robot_vel
        total_distance = np.linalg.norm(current_node - q) + np.linalg.norm(q - new_robot_position)
        constraints.append(total_distance <= traversable_len + (1 - x[j]) * traversable_len)

    # Formulate the problem
    problem = cp.Problem(objective, constraints)

    # Solve the problem
    problem.solve()

    if problem.status == cp.OPTIMAL:
        print('Optimal solution found:')
        selected_task = None
        for j in range(num_tasks):
            if x[j].value > 0.5:  # Use 0.5 as threshold since x[j] is boolean
                print(f'Task {task_set[j][0], task_set[j][1], task_set[j][2]} is selected with value: {task_set[j][4]}')
                selected_task = task_set[j][:3]
                break
        return selected_task if selected_task is not None else None, j
    else:
        print('The optimizer does not have an optimal solution.')
        return [], None

# Example usage
q = [1, 2, 3]
l1 = 1.0
t1 = 2.0
robot_position = [0, 0, 0]
robot_vel = [1, 1, 1]
uav_vel = 1.0
traversable_len = 10.0

t2_solution = optimize2(q, l1, t1, robot_position, robot_vel, uav_vel, traversable_len)
print('t2 solution:', t2_solution)

task_set = [[1, 2, 3, 0, 10], [2, 3, 4, 0, 20]]
current_node = [0, 0, 0]
robot_position = [0, 0, 0]
robot_vel = [1, 1, 1]
t_total = [1.0, 2.0]
traversable_len = 10.0

q_, pos = optimize1(task_set, current_node, robot_position, robot_vel, t_total, traversable_len)
print('q_ solution:', q_)
