"""
Maze Solver using A* Search
Syntecxhub AI Internship - Week 1, Project 1

Maze format:  S = start   G = goal   # = wall   space or . = open cell
"""

import argparse
import heapq
import math
import sys


# ---------- 1. Turn the maze text into a grid ----------

def parse_maze(text):
    """Return (grid, start, goal). In grid: 1 = wall, 0 = open cell."""
    lines = [ln for ln in text.splitlines() if ln != ""]
    if not lines:
        raise ValueError("Maze is empty.")
    width = max(len(ln) for ln in lines)
    grid, start, goal = [], None, None
    for r, line in enumerate(lines):
        row = []
        for c, ch in enumerate(line.ljust(width)):
            if ch == "#":
                row.append(1)
            else:
                row.append(0)
                if ch == "S":
                    start = (r, c)
                elif ch == "G":
                    goal = (r, c)
        grid.append(row)
    if start is None or goal is None:
        raise ValueError("Maze must contain both an 'S' (start) and a 'G' (goal).")
    return grid, start, goal


def load_maze(path):
    with open(path, "r", encoding="utf-8") as f:
        return parse_maze(f.read())


# ---------- 2. Heuristics (the "guess" of distance to the goal) ----------

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


HEURISTICS = {"manhattan": manhattan, "euclidean": euclidean}


# ---------- 3. A* search ----------

def neighbors(grid, node):
    """Open squares next to this one: up, down, left, right."""
    r, c = node
    for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        nr, nc = r + dr, c + dc
        if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]) and grid[nr][nc] == 0:
            yield (nr, nc)


def in_bounds(grid, node):
    return 0 <= node[0] < len(grid) and 0 <= node[1] < len(grid[0])


def astar(grid, start, goal, heuristic="manhattan"):
    """
    Returns (path, explored)
      path     = list of (row, col) from start to goal, or None if unreachable
      explored = squares the search looked at, in order
    """
    h = HEURISTICS[heuristic]

    for name, node in (("Start", start), ("Goal", goal)):
        if not in_bounds(grid, node):
            raise ValueError(f"{name} {node} is outside the maze.")
        if grid[node[0]][node[1]] == 1:
            raise ValueError(f"{name} {node} is on a wall.")

    counter = 0
    # heap items: (f, -g, counter, node)   where f = g + h
    open_heap = [(h(start, goal), 0, counter, start)]
    came_from = {}          # remembers where we came from, to rebuild the path
    g_score = {start: 0}    # cheapest known steps from start to each square
    closed = set()
    explored = []

    while open_heap:
        f, neg_g, _, current = heapq.heappop(open_heap)   # lowest f first
        g = -neg_g
        if current in closed:
            continue
        closed.add(current)
        explored.append(current)

        if current == goal:                                # reached the goal!
            return rebuild_path(came_from, current), explored

        for nb in neighbors(grid, current):
            new_g = g + 1                                  # each step costs 1
            if nb not in g_score or new_g < g_score[nb]:
                g_score[nb] = new_g
                came_from[nb] = current
                counter += 1
                heapq.heappush(open_heap, (new_g + h(nb, goal), -new_g, counter, nb))

    return None, explored      # nothing left to try: goal is unreachable


def rebuild_path(came_from, node):
    path = [node]
    while node in came_from:
        node = came_from[node]
        path.append(node)
    path.reverse()
    return path


# ---------- 4. Show the result ----------

def render_console(grid, start, goal, path=None, explored=None):
    """# wall, S start, G goal, * path, . explored"""
    path_set = set(path or [])
    explored_set = set(explored or [])
    out = []
    for r, row in enumerate(grid):
        line = []
        for c, cell in enumerate(row):
            pos = (r, c)
            if pos == start:
                line.append("S")
            elif pos == goal:
                line.append("G")
            elif cell == 1:
                line.append("#")
            elif pos in path_set:
                line.append("*")
            elif pos in explored_set:
                line.append(".")
            else:
                line.append(" ")
        out.append("".join(line))
    return "\n".join(out)


def plot_matplotlib(grid, start, goal, path, explored, title=""):
    """Optional picture. Needs:  pip install matplotlib"""
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    data = [row[:] for row in grid]          # 0 open, 1 wall, 2 explored, 3 path
    for r, c in explored:
        data[r][c] = 2
    for r, c in path or []:
        data[r][c] = 3
    cmap = ListedColormap(["white", "black", "#bcd4f6", "#2ecc71"])
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(data, cmap=cmap, vmin=0, vmax=3)
    ax.scatter(start[1], start[0], c="blue", s=120, marker="o", label="Start")
    ax.scatter(goal[1], goal[0], c="red", s=120, marker="*", label="Goal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title)
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.show()


def solve_and_report(grid, start, goal, heuristic, plot=False):
    try:
        path, explored = astar(grid, start, goal, heuristic)
    except ValueError as e:
        print(f"Error: {e}")
        return None

    print(render_console(grid, start, goal, path, explored))
    print()
    if path is None:
        print(f"No path found. The goal is unreachable ({len(explored)} cells explored).")
    else:
        print(f"Heuristic      : {heuristic}")
        print(f"Path length    : {len(path) - 1} steps")
        print(f"Cells explored : {len(explored)}")
        print("Legend         : S start, G goal, # wall, * path, . explored")
    if plot:
        title = f"A* ({heuristic}) - " + ("no path" if path is None else f"{len(path) - 1} steps")
        plot_matplotlib(grid, start, goal, path, explored, title)
    return path


# ---------- 5. Built-in demo maze + command line ----------

DEMO_MAZE = """\
####################
#S     #           #
# #### # ######### #
# #    #       #   #
# # ######## # # ###
# #        # # #   #
# ######## # # ### #
#        # #   #   #
######## # ##### # #
#        #       #G#
####################
"""


def main(argv=None):
    p = argparse.ArgumentParser(description="Maze solver using A* search.")
    p.add_argument("maze", nargs="?", help="maze text file (leave empty to run the demo)")
    p.add_argument("--heuristic", choices=HEURISTICS, default="manhattan")
    p.add_argument("--plot", action="store_true", help="show a matplotlib picture")
    args = p.parse_args(argv)

    try:
        grid, start, goal = load_maze(args.maze) if args.maze else parse_maze(DEMO_MAZE)
    except (OSError, ValueError) as e:
        print(f"Error loading maze: {e}")
        return 1

    path = solve_and_report(grid, start, goal, args.heuristic, args.plot)
    return 0 if path is not None else 2


if __name__ == "__main__":
    sys.exit(main())