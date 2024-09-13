# Cooperative planning for heterogeneous robots

## Overview
The project is implement a multi robots system to perform a cooperative mission in unknown environment. The system contains a ground vehicle carrying a fleet of drones performing the task of surveillance.

## Running the code
To run simulation:

1. Clone the code
2. Open it with your favorite code editor
3. Running the `main.py` file to get the simulation results


<!-- ## Citation

If you use this code, please cite the following paper:

```plaintext
@article{Duong2024NMOPSO,
  title={Navigation Variable-based Multi-objective Particle Swarm Optimization for UAV Path Planning with Kinematic Constraints},
  author={T.N. Duong, D.-N. Bui, M.D. Phung},
  journal={},
  year={2024},
  volume={},
  pages={}
} -->

## Result

![Result](dronelines.png)

** Note: there will be some cases that the planner is good, but drones maybe crash, it is because of the incompleteness of the drone controller. Modify the epsilon value in config.py file to prevent the problem.
