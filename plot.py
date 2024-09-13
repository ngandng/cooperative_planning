import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from config import save_file
# Define colors for the drones
colors = ['purple', 'orange', 'cyan']

class TrajectoryPlotter:
    def __init__(self, robot_state, drones_info, task_list, start, goals, finished_task):
        self.robot_state = np.array(robot_state)
        self.drones_info = drones_info
        self.task_list = task_list
        self.start = start
        self.goals = np.array(goals)
        self.finished = finished_task

        self.fig = plt.figure(1)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.line, = self.ax.plot([], [], [], 'b-', label='Robot Trajectory')
        self.start_marker, = self.ax.plot([], [], [], 'bo', label='Start')
        self.goals_scatter = self.ax.scatter([], [], [], c='r', marker='x', label='Goals', s=50)
        self.robot_circle, = self.ax.plot([], [], [], 'yo', label='Robot', markersize=15)
        self.robot_direction, = self.ax.plot([], [], [], 'r-')
        
        # Separate scatter plots for tasks and finished tasks
        self.task_scatter = self.ax.scatter([], [], [], c='g', marker='o', label='Task', s=30)
        self.finished_scatter = self.ax.scatter([], [], [], c='y', marker='o', label='Finished Task', s=30)

        self._init_plot()

    def _init_plot(self):
        self.ax.set_xlim((min(self.robot_state[:, 0]) - 1, max(self.robot_state[:, 0]) + 1))
        self.ax.set_ylim((min(self.robot_state[:, 1]) - 1, max(self.robot_state[:, 1]) + 1))
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
        self.robot_circle.set_data_3d([self.start[0]], [self.start[1]], [0])
        self.robot_direction.set_data_3d([], [], [])

        # Initialize goal points scatter plot
        if len(self.goals) > 0:
            self.goals_scatter._offsets3d = (self.goals[:, 0], self.goals[:, 1], self.goals[:, 2])
        else:
            self.goals_scatter._offsets3d = ([], [], [])
        
        self.task_scatter._offsets3d = ([], [], [])
        self.finished_scatter._offsets3d = ([], [], [])
        
        return (self.line, self.start_marker, self.goals_scatter, self.robot_circle, 
                self.robot_direction, self.task_scatter, self.finished_scatter)

    def update_frame(self, frame):
        self.line.set_data_3d(self.robot_state[:frame, 0], self.robot_state[:frame, 1], np.zeros(frame))
        self.robot_circle.set_data_3d([self.robot_state[frame, 0]], [self.robot_state[frame, 1]], [0])

        direction_length = 150
        direction_x = self.robot_state[frame, 0] + direction_length * np.cos(self.robot_state[frame,3])
        direction_y = self.robot_state[frame, 1] + direction_length * np.sin(self.robot_state[frame,3])
        self.robot_direction.set_data_3d([self.robot_state[frame, 0], direction_x],
                                         [self.robot_state[frame, 1], direction_y],
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

        return (self.line, self.start_marker, self.goals_scatter, self.robot_circle, 
                self.robot_direction, self.task_scatter, self.finished_scatter)

    def animate(self, filename='robot_trajectory.gif', fps=10):
        ani = FuncAnimation(self.fig, self.update_frame, frames=len(self.robot_state), 
                            init_func=self.init, blit=True, repeat=False)
        if save_file:
            ani.save(filename, writer=PillowWriter(fps=fps))
        # plt.show()

    def plot_drone_lines(self, filename1='dronelines.png',filename2='batterylog.png'):
        ## Plot for drones path
        fig = plt.figure(2,figsize=(10, 8))
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
        ax.scatter(tasks_x, tasks_y, tasks_z, c='b',s=70, marker='o')

        # Plot robot trajectory
        robot_x, robot_y, robot_z = zip(*self.robot_state[:, :3])
        ax.plot(robot_x, robot_y, robot_z, c='r', linestyle='-', linewidth=4)

        # Plot start and goal
        ax.scatter(*self.start[:3], c='green', marker='s', s=300)
        ax.scatter(self.goals[:,0],self.goals[:,1],self.goals[:,2], c='red', marker='x', s=70)

        for i, drone in enumerate(self.drones_info):
            drone_positions = drone["positions"]
            drone_x, drone_y, drone_z = zip(*[(pos[0], pos[1], pos[2]) for pos in drone_positions])
            ax.plot(drone_x, drone_y, drone_z, linestyle='--', linewidth=2, label=f'Drone {i}', color=colors[i % len(colors)])

        # Set plot labels and legend
        ax.set_xlabel('X',fontsize=18)
        ax.set_ylabel('Y',fontsize=18)
        ax.set_zlabel('Z',fontsize=18)
        # ax.set_title('Trajectory Plot')
        ax.tick_params(axis='both', labelsize=16)
        ax.legend(fontsize=18, loc='upper left', bbox_to_anchor=(0.05, 1), borderaxespad=0.)
        ax.grid(True)

        # Change the viewpoint
        ax.view_init(elev=30, azim=-60)  # Adjust the elevation and azimuth as needed

        if filename1 and save_file:
            plt.savefig(filename1)

        plt.show()


        ## 2D Plot (XY plane)
        plt.figure(5)  # Create a new figure and subplot

        # Plot task positions (2D)
        plt.scatter(tasks_x, tasks_y, c='b', s=70, marker='o')

        # Plot robot trajectory (2D)
        plt.plot(robot_x, robot_y, c='r', linestyle='-', linewidth=4)

        # Plot start and goal (2D)
        plt.scatter(self.start[0], self.start[1], c='green', marker='s', s=300)
        plt.scatter(self.goals[:, 0], self.goals[:, 1], c='red', marker='x', s=70)

        # Plot drones' paths (2D)
        for i, drone in enumerate(self.drones_info):
            drone_positions = drone["positions"]
            drone_x, drone_y = zip(*[(pos[0], pos[1]) for pos in drone_positions])  # 2D (XY only)
            plt.plot(drone_x, drone_y, linestyle='--', linewidth=2, label=f'Drone {i}', color=colors[i % len(colors)])

        # Set plot labels and legend (2D)
        plt.xlabel('X',fontsize=18)
        plt.ylabel('Y',fontsize=18)
        # plt.set_title('2D XY Trajectory Plot')
        plt.legend(fontsize=18)
        plt.grid(True)
        plt.xticks(fontsize = 16) 
        plt.yticks(fontsize = 16) 

        # Show plot
        plt.show()

        # Save the figure if needed
        if save_file:
            plt.savefig('droneline_2d.png')

        # plot for drones battery
        plt.figure(3)
        for i, drone in enumerate(self.drones_info):
            drone_battery = drone["battery"]  # Assuming this is a list of battery levels over time
            time_steps = range(len(drone_battery))  # Create a list of time steps
            plt.plot(time_steps, drone_battery, linestyle='-', linewidth=4, label=f'Drone {i}', color=colors[i % len(colors)])

        # Set plot labels and legend
        plt.xlabel('Time step',fontsize=18)
        plt.ylabel('Drone battery state (%)',fontsize=18)
        plt.title('Drones Battery Log',fontsize=18)
        plt.legend(fontsize=18)
        plt.grid(True)
        plt.xticks(fontsize = 16) 
        plt.yticks(fontsize = 16) 

        if filename2 and save_file:
            plt.savefig(filename2)

        plt.show()

    def plot_robot_vel(self,robot_vel, filename = 'robot_vel.png'):
        plt.figure()

        timesteps = range(len(robot_vel))
        plt.plot(timesteps, robot_vel, linestyle='-', linewidth=2, label=f'GV_vel')

        plt.xlabel('Time step',fontsize=18)
        plt.ylabel('Velocity of Ground Vehicle',fontsize=18)
        plt.legend(fontsize=18)
        plt.grid(True)
        plt.xticks(fontsize = 16) 
        plt.yticks(fontsize = 16) 

        if filename and save_file:
            plt.savefig(filename)

        plt.show()

