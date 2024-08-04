
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D

class TrajectoryPlotter:
    def __init__(self, trajectory, task_list, start, goal):
        self.trajectory = np.array(trajectory)
        self.task_list = task_list
        self.start = start
        self.goal = goal

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line, = self.ax.plot([], [], [], 'b-', label='Robot Trajectory')
        self.start_marker, = self.ax.plot([], [], [], 'bo', label='Start')
        self.goal_marker, = self.ax.plot([], [], [], 'ro', label='Goal')
        self.robot_circle, = self.ax.plot([], [], [], 'yo', label='Robot', markersize=20)
        self.robot_direction, = self.ax.plot([], [], [], 'r-')
        self.task_scatter = self.ax.scatter([], [], [], c='g', marker='o', label='Task', s=50)

        self._init_plot()

    def _init_plot(self):
        self.ax.set_xlim((min(self.trajectory[:, 0]) - 1, max(self.trajectory[:, 0]) + 1))
        self.ax.set_ylim((min(self.trajectory[:, 1]) - 1, max(self.trajectory[:, 1]) + 1))
        self.ax.set_zlim(0, 30)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.legend(loc='upper left')
        self.ax.set_title('Trajectory of Differential Drive Robot')
        self.ax.grid(True)

    def init(self):
        self.line.set_data_3d([], [], [])
        self.start_marker.set_data_3d([self.start[0]], [self.start[1]], [0])
        self.goal_marker.set_data_3d([self.goal[0]], [self.goal[1]], [0])
        self.robot_circle.set_data_3d([self.start[0]], [self.start[1]], [0])
        self.robot_direction.set_data_3d([], [], [])
        self.task_scatter._offsets3d = ([], [], [])
        return self.line, self.start_marker, self.goal_marker, self.robot_circle, self.robot_direction, self.task_scatter

    def update_frame(self, frame):
        self.line.set_data_3d(self.trajectory[:frame, 0], self.trajectory[:frame, 1], np.zeros(frame))
        self.robot_circle.set_data_3d([self.trajectory[frame, 0]], [self.trajectory[frame, 1]], [0])

        direction_length = 10.0
        direction_x = self.trajectory[frame, 0] + direction_length * np.cos(self.start[2])
        direction_y = self.trajectory[frame, 1] + direction_length * np.sin(self.start[2])
        self.robot_direction.set_data_3d([self.trajectory[frame, 0], direction_x],
                                         [self.trajectory[frame, 1], direction_y],
                                         [0, 0])

        if frame < len(self.task_list):
            tasks = self.task_list[frame]
            if len(tasks) > 0:
                tx = tasks[:, 0]
                ty = tasks[:, 1]
                tz = tasks[:, 2]
                self.task_scatter._offsets3d = (tx, ty, tz)
            else:
                self.task_scatter._offsets3d = ([], [], [])

        return self.line, self.start_marker, self.goal_marker, self.robot_circle, self.robot_direction, self.task_scatter

    def animate(self, filename='robot_trajectory.gif', fps=10):
        ani = FuncAnimation(self.fig, self.update_frame, frames=len(self.trajectory), init_func=self.init, blit=True, repeat=False)
        ani.save(filename, writer=PillowWriter(fps=fps))
        plt.show()