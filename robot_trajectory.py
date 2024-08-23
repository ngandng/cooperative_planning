import numpy as np
from config import *


def straight_line():
    goal = [[400, 200,0]]             # Goal position (x, y)
    return goal

def circular_path(n=10, r=70, center=(0, 0, 0)):
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

def curve_line():
    pass
