import numpy as np

from ortools.init.python import init
from ortools.linear_solver import pywraplp

def optimize1(task_set, current_node, robot_position, robot_vel, t_total, traversable_len):
    """ 
    argmax        task_set[j][4] for j in len(task_set)
    subject to    distance(current_node, q) + distance(q, new_robot) <= traversable_len
    with q = [task_set[j][0], task_set[j][1], task_set[j][2]], new_robot = robot_position + t_total[j] * robot_vel  
    """

    # Create the solver
    solver = pywraplp.Solver.CreateSolver('SCIP')
    if not solver:
        print("Could not create solver SCIP")
        return

    num_tasks = len(task_set)

    # Variables: Binary variables to select a task
    x = [solver.BoolVar(f'x[{j}]') for j in range(num_tasks)]

    # Objective: Maximize the task value
    objective = solver.Objective()
    for j in range(num_tasks):
        objective.SetCoefficient(x[j], task_set[j][4])
    objective.SetMaximization()

    # Precompute distances
    current_node = np.array(current_node)
    robot_position = np.array(robot_position)
    robot_vel = np.array(robot_vel)

    def distance(p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    M = 1e6  # A large constant for the big-M method

    for j in range(num_tasks):
        q = np.array(task_set[j][:3])
        t_total_j = t_total[j]

        # Constraint: distance(current_node, q) + distance(q, new_robot_position) <= traversable_len
        new_robot_position = robot_position + t_total_j * robot_vel
        total_distance = distance(current_node, q) + distance(q, new_robot_position)
        
        # Add the constraint using a linear form
        solver.Add(total_distance*x[j] <= traversable_len)

    solver.Add(sum(x[:]) == 1)

    # Solve
    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        # print('[LOG]: Optimal solution found:')
        selected_task = None
        for j in range(num_tasks):
            if x[j].solution_value() > 0:
                # print(f'Task {task_set[j][0],task_set[j][1],task_set[j][2]} is selected with value: {task_set[j][4]}')
                selected_task = task_set[j][:3]
                break
        return selected_task if selected_task is not None else None, j
    else:
        print('[LOG] WARNING: The optimizer1 does not have an optimal solution.')
        return [], None


def optimize2(q, l1, t1, robot_position, robot_vel, uav_vel, traversable_len):
    # Create the solver
    solver = pywraplp.Solver.CreateSolver('SCIP')
    if not solver:
        print("Could not create solver SCIP")
        return None

    # Variable
    t2 = solver.NumVar(0.00001, solver.infinity(), 't2')

    # Objective: Minimize t2 * uav_vel
    objective = solver.Objective()
    objective.SetCoefficient(t2, uav_vel)
    objective.SetMinimization()

    # Precompute constant parts of the constraint
    robot_pos = np.array(robot_position)
    robot_velocity = np.array(robot_vel)
    q_point = np.array(q)
    fixed_part = robot_pos + t1 * robot_velocity

    # Define the quadratic constraint norm((fixed_part + t2 * robot_velocity) - q) + l1 <= traversable_len
    new_position = fixed_part + t2 * robot_velocity
    distance_squared = solver.Sum([(new_position[i] - q_point[i])**2 for i in range(len(q))])
    solver.Add(distance_squared <= (traversable_len - l1) ** 2)

    # Solve the problem
    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        print('[LOG]: t2 =', t2.solution_value())
        return t2.solution_value()
    else:
        print('[LOG] WARNING: The problem does not have an optimal solution.')
        return None
    
# # Example usage
# robot_position = [0, 0]
# robot_vel = [1, 1]
# q = [5, 5]
# l1 = 1
# traversable_len = 10
# uav_vel = 2
# t1 = 2

# t2 = optimize2(q, l1, t1, robot_position, robot_vel, uav_vel, traversable_len)

# # Example usage
# task_set = [
#     [2, 3, 4, 'Task 1', 10],
#     [5, 6, 7, 'Task 2', 15],
#     [1, 1, 1, 'Task 3', 20]
# ]
# current_node = [0, 0, 0]
# robot_position = [0, 0, 0]
# robot_vel = [1, 1, 1]
# t_total = [2, 3, 1]  # Assuming t_total[j] for each task j
# traversable_len = 20

# best_task = optimize1(task_set, current_node, robot_position, robot_vel, t_total, traversable_len)
# if best_task:
#     print(f'Best task: {best_task}')