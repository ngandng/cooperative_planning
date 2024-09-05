import numpy as np
import config

def load_txt(filename):
    data = {}
    nodes = []
    goals = []
    reading_goals = False

    with open(filename, 'r') as file:
        lines = file.readlines()
        
        for line in lines:
            line = line.strip()
            if line.startswith("NAME"):
                data['name'] = line.split(":")[1].strip()
            elif line.startswith("COMMENT"):
                data['comment'] = line.split(":")[1].strip()
            elif line.startswith("DIMENSION"):
                data['dimension'] = int(line.split(":")[1].strip())
            elif line.startswith("START"):
                data['start'] = list(map(float, line.split(":")[1].strip().split()))
            elif line.startswith("UAV_MAX_TIME"):
                data['uav_max_time'] = int(line.split(":")[1].strip())
            elif line.startswith("GOAL"):
                # data['goal'] = list(map(float, line.split(":")[1].strip().split()))
                reading_goals = True
                data['goal'] = []  # Initialize the 'goal' list
                continue
            elif line.startswith("SENSIN_RANGE"):
                data['sensing_range'] = int(line.split(":")[1].strip())
            elif line.startswith("NODE_COORD_SECTION"):
                reading_goals = False
                continue  # Skip the NODE_COORD_SECTION line
            elif reading_goals:
                # Parse goal coordinates
                parts = line.split()
                x_coord = float(parts[0])
                y_coord = float(parts[1])
                goals.append([x_coord, y_coord, 0])
            else:
                # Parse node coordinates
                parts = line.split()
                node_id = int(parts[0])
                x_coord = float(parts[1])
                y_coord = float(parts[2])
                z_coord = 100                                             # here I specified the z-coordinate of tasks but it can also be provided on the file
                nodes.append([node_id, x_coord, y_coord, z_coord, 0])     # the last 0 element to be flag whether the task are noticed by the robot or not

        if len(data['start']) < 3:
            data['start'].append(0)

        for goal in goals:
            if len(goal) < 3:
                goal.append(0)
            data['goal'].append(goal)

    print('I am reading the data file with: start',data['start'],'goal',data['goal'])
    return data['start'], data['goal'], data['sensing_range'],nodes, data['uav_max_time']

def distance(a, b):
        return np.linalg.norm(np.array(a)-np.array(b))

def sense_task_from_file(environment, robot):
    probot = robot.get_position()

    # print('I am sensing for tasks: my position',probot,'my sensing range',robot.sensing_range)
    for task in environment.tasks:
        task_position = task[1:4]
        task_position[-1] = 0
        if distance(task_position,probot) < robot.sensing_range and task[-1]==0:
            newtask = np.array([task[1],task[2],task[3],0])
            robot.add_task(newtask)
            task[-1] = 1
            print('I found a new task here at position %s in the sense_task_from_file function' % (robot.get_position()))