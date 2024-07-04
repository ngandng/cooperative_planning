import numpy as np

import numpy as np

# Return the position of selected number
def roulette_wheel_selection(probabilities, num_selections=1):
    # Normalize probabilities
    probabilities = np.array(probabilities)
    probabilities /= probabilities.sum()
    
    # Use numpy's random.choice to perform the selection based on the probabilities
    selected_indices = np.random.choice(len(probabilities), size=num_selections, p=probabilities)
    
    #(Optional) Retrieve the selected probabilities
    selected_probabilities = probabilities[selected_indices]
    
    return selected_indices

def find_route_for_drone(robot, drone):
    # from robot.position, robot.vel, robot.task
    pass