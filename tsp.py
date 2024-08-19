import numpy as np

def distance(a, b):
    return np.linalg.norm(a-b)

def calculate_total_distance(route):
    total_distance = 0
    for i in range(len(route) - 1):
        total_distance += distance(route[i],route[i + 1])

    return total_distance

def two_opt(route):
    best_route = route
    best_cost = calculate_total_distance(route)
    # improved = True
    
    # while improved:
    #     improved = False
    for i in range(1, len(route) - 2):
        for j in range(i + 1, len(route) - 1):
            # Create a new route by reversing the order of nodes between i and j
            new_route = route[:i] + route[i:j+1][::-1] + route[j+1:]
            new_cost = calculate_total_distance(new_route)
            
            if new_cost < best_cost:
                best_route = new_route
                best_cost = new_cost
                # improved = True
                print('Improved by TwoOpt')

    route = best_route
    
    return best_route, best_cost

def tsp(route):
    # input as route = [p1, p2, ..., pn]
    # with p1 is robot position
    # p2, ..., pn is tasks

    if (len(route) == 1):
        return route, np.inf

    tsp_route, tsp_cost = two_opt(route)

    return tsp_route, tsp_cost