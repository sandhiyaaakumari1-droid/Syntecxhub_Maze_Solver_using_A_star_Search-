# A* Maze Solver

A Python implementation of the A* search algorithm to find the shortest path through a 2D grid maze[cite: 5]. Built for Week 1 of the Syntecxhub AI Internship[cite: 5].

## What It Does

- Finds the shortest path between a start (`S`) and goal (`G`) point on a grid[cite: 5, 15].
- Avoids wall obstacles (`#`)[cite: 5, 15].
- Supports both **Manhattan** and **Euclidean** distance heuristics[cite: 5, 15].
- Handles edge cases where no path exists to the goal[cite: 5, 15].
- Prints the grid and final route directly in the terminal[cite: 5, 15].
- Includes an optional visual plot using `matplotlib`[cite: 5, 15].

## How to Run

### 1. Default Demo
Runs the solver on the built-in demo grid:
```bash
python maze_solver.py
