import numpy as np
from robot import *
from main import *

env = Environment()

def test_robot_initialization():
    robot = DifferentialDriveRobot(x=0, y=0, theta=0, vmax=1.0, sr=5.0)
    assert robot.x == 0         # assert raise an AssertionError when condition returns false
    assert robot.y == 0
    assert robot.theta == 0
    assert robot.vmax == 1.0
    assert robot.sensing_range == 5.0
    print("Robot initialization test passed.")

def test_update_position():
    robot = DifferentialDriveRobot(x=0, y=0, theta=0, vmax=1.0, sr=5.0)
    robot.update_position(v=1, omega=0.1, dt=1, env=env)
    assert np.isclose(robot.x, 1)
    assert np.isclose(robot.y, 0)
    assert np.isclose(robot.theta, 0.1)
    print("Update position test passed.")

def test_add_task():
    robot = DifferentialDriveRobot(x=0, y=0, theta=0, vmax=1.0, sr=5.0)
    task = [1, 2, 3, 0, 0.5]
    robot.add_task(task)
    assert robot.task.shape == (1, 5)
    print("Add task test passed.")

def test_sense_new_task():
    class DummyEnv:
        zmax = 10

    robot = DifferentialDriveRobot(x=0, y=0, theta=0, vmax=1.0, sr=5.0)
    robot.sense_new_task(DummyEnv())
    assert robot.task.shape[1] == 5
    print("Sense new task test passed.")

def test_calculate_priority():
    robot = DifferentialDriveRobot(x=0, y=0, theta=0, vmax=1.0, sr=5.0)
    robot.add_task([1, 1, 1, 0, 0.5])
    robot.add_task([2, 2, 2, 0, 0.5])
    robot.update_vel([1, 0, 0])
    robot.calculate_priority()
    assert robot.task.shape == (2, 5)
    print("Calculate priority test passed.")

def test_plan_for_uav():
    class DummyEnv:
        zmax = 10

    robot = DifferentialDriveRobot(x=0, y=0, theta=0, vmax=1.0, sr=5.0)
    robot.add_task([1, 1, 1, 0, 0.5])
    robot.add_task([2, 2, 2, 0, 0.5])
    robot.update_vel([1, 0, 0])
    robot.calculate_priority()
    uav_route = robot.plan_for_uav(uav_vel=1.0, uav_battery=20)
    print("Planned UAV route:", uav_route)

if __name__ == "__main__":
    test_robot_initialization()
    test_update_position()
    test_add_task()
    test_sense_new_task()
    test_calculate_priority()
    test_plan_for_uav()
