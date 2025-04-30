import random
from cspace import ConfigSpace
import numpy as np

class RRT:
    def __init__(self, q_start, q_goal, cspace: ConfigSpace, n_iters=1000, step_size=0.5, p=0.2):
        """
        Initialize the RRT planner.

        Args:
            q_start: Starting node
            q_goal: Goal node
            cspace: Configuration space
            n_iters: max number of iterations to build the tree
            step_size: max distance to extend tree at each iteration
            p: probability of sampling the goal directly (goal_bias)
        """
        self.start = q_start
        self.goal = q_goal
        self.cspace = cspace
        self.n_iters = n_iters
        self.step_size = step_size
        self.goal_bias = p

        self.V = [q_start]  # list of nodes in the tree
        self.E = []         # list of edges as (start, end, weight)
        self.parent = {tuple(self.start) : None}    # maps each node to its parent for path reconstruction

    def sample(self):
        """
        Sample a random node in the configuration space.
        """
        if random.random() < self.goal_bias:
            return self.goal
        return self.cspace.sample()
    
    def closest_neighbor(self, q_rand):
        """
        Find the closest existing node in the tree to the randomly sampled node.

        Args:
            q_rand: Randomly sampled node
        
        Returns:
            the closest node in the tree
        """
        closest_node, closest_dist = None, float("inf")
        q_rand = np.array(q_rand, dtype=float)
        for q_node in self.V:
            q_node = np.array(q_node, dtype=float)
            dist = np.linalg.norm(q_rand-q_node)
            if dist < closest_dist:
                closest_node, closest_dist = q_node, dist
        return closest_node
    
    def drive_to(self, q_from, q_to):
        """
        Move from q_from toward q_to by at most 'step_size'.

        Args:
            q_from: Starting node
            q_to: Target node

        Returns:
            New node moved toward q_to by step_size
        """
        dx, dy = q_to[0] - q_from[0], q_to[1] - q_from[1]
        dist = np.linalg.norm(q_from - q_to)

        if dist == 0:
            return q_to
        
        scale = min(self.step_size, dist)/dist
        u = (dx*scale, dy*scale)

        q_new = q_from + u
        return q_new

    def extend(self, q_rand):
        """
        Try to expand the tree toward a sampled random node.

        Args:
            q_rand: Randomly sampled node.
        """
        q_near = self.closest_neighbor(q_rand)  # Find nearest tree node
        q_new = self.drive_to(q_near, q_rand)   # Step towards q_rand 

        # Add q_new only if it is in the configuration space and path is collision-free
        if self.cspace.is_in_bounds(q_new) and self.cspace.collision_free(q_near, q_new):
            q_new_t = tuple(q_new)
            q_near_t = tuple(q_near)

            # Avoid adding duplicate nodes
            if q_new_t == q_near_t:
                return
            
            # Node already exists in the tree
            if q_new_t in self.parent:
                return

            # Add new node and edge to the tree
            self.V.append(q_new_t)
            self.E.append((q_near_t, q_new_t, self.step_size))
            self.parent[q_new_t] = q_near_t
    
    def build(self):
        """
        Build the RRT by iteratively sampling and extending.

        Returns:
            the list of nodes and edges
        """
        for _ in range(self.n_iters):
            q_rand = self.sample()
            self.extend(q_rand)

        return self.V, self.E
    
    def get_path(self):
        """
        Reconstruct the path from start to goal after building the tree.

        Returns:
            a list of nodes from start node to goal node (or closest node to goal node)
        """
        start = tuple(self.start)
        goal = tuple(self.goal)

        # Find the actual goal node it it was reached
        if goal in self.parent:
            end = goal
        else:
            # Otherwise find the closest node to the goal
            end, closest_dist = None, float("inf")

            for q in self.V:
                dist = np.linalg.norm(np.array(q) - np.array(goal))
                if dist < closest_dist:
                    end, closest_dist = q, dist

            end = tuple(end)

        # Backtrack from end node to start node using parent backpointers
        path = []
        current = end
        max_steps = len(self.V) + 1

        for _ in range(max_steps):
            path.append(current)
            if current == start:
                break

            current = self.parent.get(current)
            if current is None:
                raise RuntimeError(f"Lost parent link when backtracking from {current}")
            
        path.reverse()  # Reverse the path to start -> goal 
        return path

