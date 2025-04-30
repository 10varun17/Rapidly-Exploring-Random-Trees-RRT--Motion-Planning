# RRT-Based Robot Motion Planning and Simulation

## Overview

This project implements a sampling-based Rapidly-exploring Random Tree (RRT) planner and simulates a unicycle‐model robot navigating in a 2D configuration space with circular obstacles.

---

### Running the Demo

From the project root, run:

```bash
python main.py
```

This will:

1. Initialize a random set of circular obstacles in the workspace (via `ConfigSpace`).
2. Build an RRT from the start to the goal with a goal‐bias sampling strategy.
3. Reconstruct the tree path and simulate a unicycle robot following it.
4. Display two subplots:
   - The RRT tree, tree nodes, obstacles, and final path.
   - The robot’s actual trajectory tracking the planned waypoints.

---

## File Structure

- **cspace.py**  
  Defines `ConfigSpace`, which manages C-space bounds and randomly placed circular obstacles. Provides collision checking and uniform sampling.

- **rrt.py**  
  Implements the `RRT` class:  
  - `sample()`: with goal bias.  
  - `closest_neighbor()`, `drive_to()`, `extend()` to grow the tree.  
  - `build()` to run iterations and return the nodes and edges in the tree.  
  - `get_path()` to get a path from start node to goal node or the nearest node to the goal.

- **robot.py**  
  Defines a simple unicycle‐model `Robot` with state \((x, y, yaw)\).  
  - `update_state(u)`: integrates velocity \(v\) and angular rate \(\omega\) to advance the robot.

- **visualize.py**  
  Contains:
  - `visualize_rrt()`: plots tree edges, nodes, obstacles, start/goal markers, and the planned path.
  - `visualize_traj()`: overlays the robot’s executed trajectory on the same obstacle map.

- **main.py**  
  Coordinates everything:  
  1. Creates `ConfigSpace` and `Robot`.  
  2. Builds the RRT and retrieves the path.  
  3. Simulates the robot using a nonlinear control transform to follow waypoints.  
  4. Calls visualization functions and shows/saves the resulting figures.
