import numpy as np
from cspace import ConfigSpace
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def visualize_rrt(V, E, start, goal, cspace:ConfigSpace, path, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(6,6))

    V = [tuple(q) for q in V]
    E = [(tuple(q_near), tuple(q_new), s) for q_near, q_new, s in E]

    # Plot tree edges
    for q_from, q_to, _ in E:
        ax.plot([q_from[0], q_to[0]], [q_from[1], q_to[1]], c="black", linewidth=1, linestyle="--", alpha=0.4)

    # Plot tree nodes
    xs, ys = zip(*V)
    ax.scatter(xs, ys, s=10, alpha=0.6, label="tree nodes", c="black")

    # Highlight start and goal
    ax.scatter([start[0]], [start[1]], c="red", s=100, marker="*", label="start")
    ax.scatter([goal[0]], [goal[1]], c="green", s=100, marker="*", label="goal")

    # Draw obstacles
    for center, radius in cspace.obstacles:
        circ = Circle(center, radius, edgecolor="purple", facecolor="None", linewidth=2, alpha=0.7)
        ax.add_patch(circ)

    if goal in V:
        ax.plot(*zip(*path), "b-", linewidth=3, label="path to goal")
    else:
        q_closest = path[-1]
        ax.plot(*zip(*path), "b-", linewidth=3, label="sub-optimal path")
        ax.plot([q_closest[0], goal[0]], [q_closest[1], goal[1]], "r--", linewidth=2, label="to goal")

    ax.set_xlim(cspace.xmin, cspace.xmax)
    ax.set_ylim(cspace.ymin, cspace.ymax)
    ax.set_aspect("equal", "box")
    ax.set_title("RRT Visualization")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True)
    ax.legend()
    return ax

def visualize_traj(traj, cspace:ConfigSpace, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(6,6))

    xs, ys = zip(*traj)

    # Draw obstacles
    for center, radius in cspace.obstacles:
        circ = Circle(center, radius, edgecolor="purple", facecolor="None", linewidth=2, alpha=0.7)
        ax.add_patch(circ)

    ax.plot(xs, ys, "b-", label="robot path")
    ax.scatter(xs[0], ys[0], c="red", s=80, marker="*", label="start")
    ax.scatter(xs[-1], ys[-1], c="green", s=80, marker="*", label="end")
    ax.set_xlim(cspace.xmin, cspace.xmax)
    ax.set_ylim(cspace.ymin, cspace.ymax)
    ax.set_aspect("equal", "box")
    ax.grid(True)
    ax.legend()
    ax.set_title("Robot following planned path")
    return ax