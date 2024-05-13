# Robot Navigation Problem Solver

This Python program provides a comprehensive framework for solving robot navigation problems in a grid environment with obstacles and multiple goal states using various search algorithms. The implemented algorithms include:

- Greedy Best-First Search (GBFS)
- A* Search (AS)
- Depth-First Search (DFS)
- Breadth-First Search (BFS)
- Iterative Deepening Search (IDS)
- Directional A* Search (DAS)
- Multi-Goal Breadth-First Search (MULTIBFS)

## Table of Contents

- [Features](#features)
- [Setup](#setup)
- [Usage](#usage)
- [Search Algorithms](#search-algorithms)
- [Grid Data Format](#grid-data-format)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

## Features

- **Abstract Problem Class**: Provides a base class for defining problem-specific state spaces, actions, and goal tests.
- **Search Algorithms**: Implements various search algorithms for solving grid-based navigation problems.
- **Timing and Node Expansion Tracking**: Each search method returns the number of nodes expanded and the runtime, providing insights into the performance of each algorithm.
- **Grid Data Parsing**: Reads and parses grid data from a file to set up the problem environment.

## Ensure you have Python 3 installed. Install any required dependencies (if any):

    -pip install -r requirements.txt

##Usage

    -To run the script, use the following command:
    -python script.py <filename> <method>

    <filename>: Path to the file containing the grid data.
    <method>: The search algorithm to use. Valid methods are: GBFS, AS, DFS, BFS, CU1, CU2, MULTIBFS.

##Example:

    python script.py grid.txt GBFS

##Search Algorithms

--Greedy Best-First Search (GBFS)
Searches the nodes with the lowest heuristic scores first. Uses a heuristic function to estimate the cost to the nearest goal.

--A* Search (AS)
Combines the cost to reach the node and the heuristic cost to estimate the total path cost.

--Depth-First Search (DFS)
Explores as far as possible along each branch before backing up.

--Breadth-First Search (BFS)
Explores all nodes at the present depth level before moving on to nodes at the next depth level.

--Iterative Deepening Search (IDS)
Combines the benefits of depth-first search and breadth-first search, exploring nodes by increasing depth limits.

--Directional A* Search (DAS)
A variant of A* search that uses directional heuristics to guide the search.

--Multi-Goal Breadth-First Search (MULTIBFS)
Finds the shortest path that visits all specified goal states in a grid environment.

##Grid Data Format
The grid data file should follow this format:

The first line contains the number of rows and columns in the grid.
The second line contains the starting position of the robot.
The third line contains the goal positions.
Subsequent lines define the walls as rectangles.

##Example:

    5, 5
    (0, 0)
    (4, 4) | (1, 3)
    (1, 1, 2, 1)
    (3, 2, 1, 3)
    
    5, 5: Grid of 5 rows and 5 columns.
    (0, 0): Starting position at (0, 0).
    (4, 4), (1, 3): Goal positions at (4, 4) and (1, 3).
    (1, 1, 2, 1): Wall starting at (1, 1) with width 2 and height 1.
    (3, 2, 1, 3): Wall starting at (3, 2) with width 1 and height 3.

##Examples
Running A* Search

    python script.py grid.txt AS

Running Multi-Goal BFS

    python script.py grid.txt MULTIBFS

##Output
The program will print the following information:

--The filename and method used.
--Whether the goal was reached or not.
--The sequence of actions taken to reach the goal.
--The number of nodes expanded.
--The runtime in seconds.

Example Output

    grid.txt AS
    goal (4, 4) number_of_nodes 10
    UP RIGHT DOWN LEFT
    Nodes expanded: 15
    Runtime: 0.0054321 seconds
