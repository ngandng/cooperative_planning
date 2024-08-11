import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D

# Define colors for the drones
colors = ['purple', 'orange', 'cyan']

class TrajectoryPlotter:
    def __init__(self, trajectory, drones_info, task_list, start, goal, finished_task):
        self.trajectory = np.array(trajectory)
        self.drones_info = drones_info
        self.task_list = task_list
        self.start = start
        self.goal = goal
        self.finished = finished_task

        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line, = self.ax.plot([], [], [], 'b-', label='Robot Trajectory')
        self.start_marker, = self.ax.plot([], [], [], 'bo', label='Start')
        self.goal_marker, = self.ax.plot([], [], [], 'ro', label='Goal')
        self.robot_circle, = self.ax.plot([], [], [], 'yo', label='Robot', markersize=20)
        self.robot_direction, = self.ax.plot([], [], [], 'r-')
        
        # Separate scatter plots for tasks and finished tasks
        self.task_scatter = self.ax.scatter([], [], [], c='g', marker='o', label='Task', s=30)
        self.finished_scatter = self.ax.scatter([], [], [], c='y', marker='o', label='Finished Task', s=30)

        self._init_plot()

    def _init_plot(self):
        self.ax.set_xlim((min(self.trajectory[:, 0]) - 1, max(self.trajectory[:, 0]) + 1))
        self.ax.set_ylim((min(self.trajectory[:, 1]) - 1, max(self.trajectory[:, 1]) + 1))
        self.ax.set_zlim(0, 30)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.legend(loc='upper left')
        self.ax.set_title('GV with tasks')
        self.ax.grid(True)

    def init(self):
        self.line.set_data_3d([], [], [])
        self.start_marker.set_data_3d([self.start[0]], [self.start[1]], [0])
        self.goal_marker.set_data_3d([self.goal[0]], [self.goal[1]], [0])
        self.robot_circle.set_data_3d([self.start[0]], [self.start[1]], [0])
        self.robot_direction.set_data_3d([], [], [])
        
        self.task_scatter._offsets3d = ([], [], [])
        self.finished_scatter._offsets3d = ([], [], [])
        
        return (self.line, self.start_marker, self.goal_marker, self.robot_circle, 
                self.robot_direction, self.task_scatter, self.finished_scatter)

    def update_frame(self, frame):
        self.line.set_data_3d(self.trajectory[:frame, 0], self.trajectory[:frame, 1], np.zeros(frame))
        self.robot_circle.set_data_3d([self.trajectory[frame, 0]], [self.trajectory[frame, 1]], [0])

        direction_length = 10.0
        direction_x = self.trajectory[frame, 0] + direction_length * np.cos(self.start[2])
        direction_y = self.trajectory[frame, 1] + direction_length * np.sin(self.start[2])
        self.robot_direction.set_data_3d([self.trajectory[frame, 0], direction_x],
                                         [self.trajectory[frame, 1], direction_y],
                                         [0, 0])

        # Update task scatter plot
        if frame < len(self.task_list):
            tasks = self.task_list[frame]
            if len(tasks) > 0:
                tx = tasks[:, 0]
                ty = tasks[:, 1]
                tz = tasks[:, 2]
                self.task_scatter._offsets3d = (tx, ty, tz)
            else:
                self.task_scatter._offsets3d = ([], [], [])
        
        # Update finished task scatter plot
        if frame < len(self.finished):
            finished_tasks = self.finished[frame]
            if len(finished_tasks) > 0:
                ftx = finished_tasks[:, 0]
                fty = finished_tasks[:, 1]
                ftz = finished_tasks[:, 2]
                self.finished_scatter._offsets3d = (ftx, fty, ftz)
            else:
                self.finished_scatter._offsets3d = ([], [], [])

        return (self.line, self.start_marker, self.goal_marker, self.robot_circle, 
                self.robot_direction, self.task_scatter, self.finished_scatter)

    def animate(self, filename='robot_trajectory.gif', fps=10):
        ani = FuncAnimation(self.fig, self.update_frame, frames=len(self.trajectory), 
                            init_func=self.init, blit=True, repeat=False)
        ani.save(filename, writer=PillowWriter(fps=fps))
        # plt.show()

    def plot_drone_lines(self, filename='dronelines.png'):
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Flatten the task list and finished task list
        all_tasks = [task for sublist in self.task_list for task in sublist]
        all_finished = [task for sublist in self.finished for task in sublist]

        # Combine tasks and finished tasks
        all_task = all_tasks + all_finished

        # Extract task positions
        if all_task:
            tasks_x, tasks_y, tasks_z = zip(*[(task[0], task[1], task[2]) for task in all_task])
        else:
            tasks_x, tasks_y, tasks_z = [], [], []

        # Plot task positions
        ax.scatter(tasks_x, tasks_y, tasks_z, c='b', marker='o', label='Tasks')

        # Plot robot trajectory
        robot_x, robot_y, robot_z = zip(*self.trajectory[:, :3])
        ax.plot(robot_x, robot_y, robot_z, c='r', linestyle='-', linewidth=2, label='Robot Trajectory')

        # Plot start and goal
        ax.scatter(*self.start[:3], c='green', marker='s', s=100, label='Start')
        ax.scatter(*self.goal[:3], c='red', marker='s', s=100, label='Goal')

        for i, drone in enumerate(self.drones_info):
            drone_positions = drone["positions"]
            drone_x, drone_y, drone_z = zip(*[(pos[0], pos[1], pos[2]) for pos in drone_positions])
            ax.plot(drone_x, drone_y, drone_z, linestyle='-', linewidth=1.5, label=f'Drone {i}', color=colors[i % len(colors)])

        # Set plot labels and legend
        ax.set_xlabel('X-coordinate')
        ax.set_ylabel('Y-coordinate')
        ax.set_zlabel('Z-coordinate')
        ax.set_title('Trajectory Plot')
        ax.legend()
        ax.grid(True)

        # Change the viewpoint
        ax.view_init(elev=30, azim=-60)  # Adjust the elevation and azimuth as needed

        if filename:
            plt.savefig(filename)

        plt.show()

    def plot_battery_info(self, filename='battery_log.png'):
        fig = plt.figure()


        for i, drone in enumerate(self.drones_info):
            drone_battery = drone["battery"]  # Assuming this is a list of battery levels over time
            time_steps = range(len(drone_battery))  # Create a list of time steps
            plt.plot(time_steps, drone_battery, linestyle='-', linewidth=2, label=f'Drone {i}', color=colors[i % len(colors)])

        # Set plot labels and legend
        plt.xlabel('Time step')
        plt.ylabel('Drone battery state (%)')
        plt.title('Drones Battery Log')
        plt.legend()
        plt.grid(True)

        if filename:
            plt.savefig(filename)

        plt.show()
