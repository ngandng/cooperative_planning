import numpy as np
from config import *


def straight_line():
    goal = [[10000, 10000,0]]             # Goal position (x, y)
    return goal

def circular_path(n=10, r=1000, center=(0, 0, 0)):
    """
    Generate n points on a circular path.
    n: Number of points to generate.
    r: Radius of the circle (default is 40).
    center: Center of the circle as a tuple (x_c, y_c, z_c).
    """
    x_c, y_c, z_c = center
    points = []

    # Calculate angles for n points
    angles = np.linspace(0, 2 * np.pi, n, endpoint=False)

    # Generate points on the circle
    for angle in angles:
        x = r * np.cos(angle) + x_c
        y = r * np.sin(angle) + y_c
        z = z_c
        points.append([x, y, z])

    return np.array(points)

def u_curve(start=start):
    
    points = []

    d = 5000  # distance between 2 edges of u-curve
    l = 20000  # length of 2 edges of u-curve

    n = 30  # total number of waypoints

    # straight line of u-curve
    for i in range (int(n/3)):
        new_wp = [0, 0, 0]
        new_wp[0] = start[0]
        new_wp[1] = start[1] + i*l/(n/3)
        new_wp[2] = 0

        points.append(new_wp)

    # the curve part
    xc = start[0] + d/2
    yc = start[1] + l
    zc = 0
    
    angles = np.linspace(np.pi, 0, int(n/3), endpoint=False)
    r = d/2
    
    for angle in angles:
        x = r * np.cos(angle) + xc
        y = r * np.sin(angle) + yc
        z = zc
        points.append([x, y, z])

    # the other straight line    
    for i in range (int(n/3)):
        new_wp = [0, 0, 0]
        new_wp[0] = start[0] + d
        new_wp[1] = start[1] + (l - i*l/(n/3))
        new_wp[2] = 0

        points.append(new_wp)

    return points

