import matplotlib.pyplot as plt

def read_and_plot(filename):
    with open(filename, 'r') as file:
        lines = file.readlines()
    
    # Parse NODE_COORD_SECTION
    coordinates = []
    goals = []
    in_node_section = False
    in_goal_section = False

    for line in lines:
        line = line.strip()
        
        # Identify sections
        if line == 'NODE_COORD_SECTION':
            in_node_section = True
            in_goal_section = False
            continue
        if line == 'GOAL':
            in_node_section = False
            in_goal_section = True
            continue
        if in_node_section and line == 'EOF':
            break
        if in_node_section:
            parts = line.split()
            if len(parts) == 3:
                node_id, x, y = parts
                coordinates.append((float(x), float(y)))
        if in_goal_section:
            parts = line.split()
            if len(parts) == 2:
                x, y = parts
                goals.append((float(x), float(y)))
    
    # Extract x and y coordinates
    x_coords, y_coords = zip(*coordinates) if coordinates else ([], [])
    goal_x_coords, goal_y_coords = zip(*goals) if goals else ([], [])
    
    # Plot the coordinates
    plt.figure(figsize=(12, 10))
    
    # Plot nodes
    plt.scatter(x_coords, y_coords, color='blue', label='Nodes')
    
    # Plot goals
    plt.scatter(goal_x_coords, goal_y_coords, color='red', marker='x', label='Goals')

    # Connect goals with lines
    if len(goals) > 1:
        for i in range(len(goals) - 1):
            x_values = [goals[i][0], goals[i + 1][0]]
            y_values = [goals[i][1], goals[i + 1][1]]
            plt.plot(x_values, y_values, color='red', linestyle='-', linewidth=2)

    # Plot start goal as a green rectangle
    if goals:
        start_goal = goals[0]
        plt.scatter(start_goal[0], start_goal[1], color='green', marker='s', s=300, label='Start')


    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('U-curve path')
    plt.legend()
    plt.grid(True)
    plt.show()


# Replace 'your_file.txt' with the path to your text file
read_and_plot('ucur1.txt')
