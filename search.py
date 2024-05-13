import sys
import time
from collections import deque, namedtuple

from utils import *

class Problem:
    """The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""

    def __init__(self, initial, goal=None):
        """The constructor specifies the initial state, and possibly a goal
        state, if there is a unique goal. Your subclass's constructor can add
        other arguments."""
        self.initial = initial
        self.goal = goal


    def actions(self, state):
        """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
        raise NotImplementedError

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
        raise NotImplementedError

    def goal_test(self, state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal or checks for state in self.goal if it is a
        list, as specified in the constructor. Override this method if
        checking against a single self.goal is not enough."""
        if isinstance(self.goal, list):
            return is_in(state, self.goal)
        else:
            return state == self.goal

    def path_cost(self, c, state1, action, state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2. If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def value(self, state):
        """For optimization problems, each state has a value. Hill Climbing
        and related algorithms try to maximize this value."""
        raise NotImplementedError

    def get_max_depth(self):

        return float('inf')

# ______________________________________________________________________________


class Node:
    """A node in a search tree. Contains a pointer to the parent (the node
    that this is a successor of) and to the actual state for this node. Note
    that if a state is arrived at by two paths, then there are two nodes with
    the same state. Also includes the action that got us to this state, and
    the total path_cost (also known as g) to reach the node. Other functions
    may add an f and h value; see best_first_graph_search and astar_search for
    an explanation of how the f and h values are handled. You will not need to
    subclass this class."""

    def __init__(self, state, parent=None, action=None, path_cost=0):
        """Create a search tree Node, derived from a parent by an action."""
        self.state = state
        self.parent = parent
        self.action = action
        self.path_cost = path_cost
        self.depth = 0
        if parent:
            self.depth = parent.depth + 1

    def __repr__(self):
        return "<Node {}>".format(self.state)

    def __lt__(self, node):
        return self.state < node.state

    def expand(self, problem):
        """List the nodes reachable in one step from this node."""
        return [self.child_node(problem, action)
                for action in problem.actions(self.state)]

    def child_node(self, problem, action):
        """[Figure 3.10]"""
        next_state = problem.result(self.state, action)
        next_node = Node(next_state, self, action, problem.path_cost(self.path_cost, self.state, action, next_state))
        return next_node

    def solution(self):
        """Return the sequence of actions to go from the root to this node."""
        return [node.action for node in self.path()[1:]]

    def path(self):
        """Return a list of nodes forming the path from the root to this node."""
        node, path_back = self, []
        while node:
            path_back.append(node)
            node = node.parent
        return list(reversed(path_back))

    # We want for a queue of nodes in breadth_first_graph_search or
    # astar_search to have no duplicated states, so we treat nodes
    # with the same state as equal. [Problem: this may not be what you
    # want in other contexts.]

    def __eq__(self, other):
        return isinstance(other, Node) and self.state == other.state

    def __hash__(self):
        # We use the hash value of the state
        # stored in the node instead of the node
        # object itself to quickly search a node
        # with the same state in a Hash Table
        return hash(self.state)


def best_first_search(problem, f, display=False):
    """Search the nodes with the lowest f scores first.
    You specify the function f(node) that you want to minimize; for example,
    if f is a heuristic estimate to the goal, then we have greedy best
    first search; if f is node.depth then we have breadth-first search.
    There is a subtlety: the line "f = memoize(f, 'f')" means that the f
    values will be cached on the nodes as they are computed. So after doing
    a best first search you can examine the f values of the path returned."""
    start_time = time.perf_counter()  # Start timing the search
    f = memoize(f, 'f')
    node = Node(problem.initial)
    frontier = PriorityQueue('min', f)
    frontier.append(node)
    explored = set()
    expanded_nodes = 0  # Initialize node expansion counter

    while frontier:
        node = frontier.pop()
        explored.add(node.state)
        expanded_nodes += 1  # Increment the node expansion counter

        if problem.goal_test(node.state):
            end_time = time.perf_counter()  # End timing on finding a solution
            return node, expanded_nodes, end_time - start_time

        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)
            elif child in frontier:
                if f(child) < frontier[child]:
                    del frontier[child]
                    frontier.append(child)

    end_time = time.perf_counter()  # End timing if no solution is found
    return None, expanded_nodes, end_time - start_time

# ______________________________________________________________________________
# Informed (Heuristic) Search


def greedy_best_first_search(problem, h=None, display=False):
    """
    Greedy best-first search is accomplished by specifying f(n) = h(n).

    Args:
        problem: The problem instance.
        h: Heuristic function used to estimate the cost to the nearest goal.
        display: If true, displays detailed search info.

    Returns:
        tuple: Node, number of nodes expanded, and runtime.
    """
    h = memoize(h or problem.heuristic, 'h')
    return best_first_search(problem, lambda n: h(n), display)

#_______________________________________________________________________________

def astar_search(problem, h=None, display=False):
    """
    A* search is best-first graph search with f(n) = g(n)+h(n).

    Args:
        problem: The problem instance.
        h: Heuristic function.
        display: If true, displays detailed search info.

    Returns:
        tuple: Node, number of nodes expanded, and runtime.
    """
    h = memoize(h or problem.heuristic, 'h')
    return best_first_search(problem, lambda n: n.path_cost + h(n), display)


#_______________________________________________________________________________
def depth_first_search(problem):
    """
    Performs a Depth First Search on the specified problem.

    Args:
        problem: An instance of the Problem class that defines the methods:
                 - initial: the initial state
                 - goal_test: function to check if the state is a goal state
                 - actions: function to determine possible actions from a state
                 - result: function to determine the resulting state from an action

    Returns:
        Node: A Node object representing the solution state, or None if no solution is found.
    """
    # Initialize the frontier with the initial state of the problem
    start_time = time.perf_counter()  # Start timing the search
    initial_node = Node(problem.initial)
    frontier = deque([initial_node])  # Use deque as a stack, pop from the end
    explored = set()
    expanded_nodes = 0  # Initialize node expansion counter

    while frontier:
        node = frontier.pop()  # Pop from the stack to get the last added node
        explored.add(node.state)
        expanded_nodes += 1  # Increment the node expansion counter

        if problem.goal_test(node.state):
            end_time = time.perf_counter()  # End timing on finding a solution
            return node, expanded_nodes, end_time - start_time

        # Expand the node in the order of UP, LEFT, DOWN, RIGHT
        for action in reversed(problem.actions(node.state)):
            child_state = problem.result(node.state, action)
            child_node = Node(child_state, node, action, problem.path_cost(node.path_cost, node.state, action, child_state))

            if child_state not in explored and child_node not in frontier:
                frontier.append(child_node)

    end_time = time.perf_counter()  # End timing if no solution is found
    return None, expanded_nodes, end_time - start_time

#_______________________________________________________________________________
# Breadth First Search
def breadth_first_search(problem):
  """
  Solves the problem using Breadth-First Search.

  Args:
      problem: An instance of the RoboNavProblem class.

  Returns:
      A Node object representing the solution state, or None if no solution exists.
  """
  start_time=time.perf_counter()  # Start the timer
  frontier = deque([Node(problem.initial)])
  explored = set()
  expanded_nodes=0
  while frontier:
        node = frontier.popleft()
        explored.add(node.state)
        expanded_nodes += 1

        if problem.goal_test(node.state):
            end_time = time.perf_counter()  # End the timer when the goal is found
            return node, expanded_nodes, end_time - start_time

        for child in node.expand(problem):
            if child.state not in explored and child not in frontier:
                frontier.append(child)

  end_time = time.perf_counter()  # End the timer if no solution is found
  return None, expanded_nodes, end_time - start_time

#_______________________________________________________________________________


def iterative_deepening_search(problem):
    """
    Performs Iterative Deepening Search.

    Args:
        problem: An instance of the Problem class with necessary methods defined.

    Returns:
        Node: A Node object representing the solution state, or None if no solution is found.
        int: Total number of nodes expanded.
        float: Total runtime of the search in seconds.
    """
    depth = 100
    expanded_nodes = 0
    start_time = time.perf_counter()  # End the timer if no solution is found

    while True:
        result, expansions = depth_limited_search(problem, depth)
        expanded_nodes += expansions
        if result is not None and result != 'cutoff':
            runtime = time.perf_counter() - start_time
            return result, expanded_nodes, runtime
        depth += 1
        if depth > problem.get_max_depth():  # Prevent infinite loops in infinite-depth spaces
            runtime = time.perf_counter() - start_time
            return None, expanded_nodes, runtime

def depth_limited_search(problem, limit):
    """
    Depth-limited search algorithm.

    Args:
        problem: The problem instance.
        limit: The maximum depth limit for the search.

    Returns:
        Node: A Node object if a solution is found, or None if no solution is found within the depth limit.
        int: Number of node expansions.
    """
    return recursive_dls(Node(problem.initial), problem, limit, 0)

def recursive_dls(node, problem, limit, expansions):
    """
    Helper function to perform Depth-Limited Search recursively.

    Args:
        node: The current node in the search tree.
        problem: The problem instance.
        limit: The current depth limit.
        expansions: Count of nodes expanded up to this point.

    Returns:
        Node: A Node object if a solution is found, or None if no solution is found.
        int: Updated number of node expansions.
    """
    if problem.goal_test(node.state):
        return node, expansions
    elif limit <= 0:
        return 'cutoff', expansions
    else:
        cutoff_occurred = False
        for action in problem.actions(node.state):
            child = node.child_node(problem, action)
            result, new_expansions = recursive_dls(child, problem, limit - 1, expansions + 1)
            expansions = new_expansions
            if result == 'cutoff':
                cutoff_occurred = True
            elif result is not None:
                return result, expansions
        return ('cutoff' if cutoff_occurred else None), expansions


#_______________________________________________________________________________

def directional_astar_search(problem):
    def heuristic(node):
        # Heuristic: Euclidean distance to the closest goal
        return min(((node.state[0] - goal[0]) ** 2 + (node.state[1] - goal[1]) ** 2) ** 0.5 for goal in problem.goal)

    start_time = time.perf_counter()  # Start timing
    frontier = PriorityQueue()
    frontier.append((0, Node(problem.initial)))
    explored = set()
    node_expanded_count = 0  # Initialize node expansion count

    while not frontier.empty():
        _, node = frontier.pop()
        if problem.goal_test(node.state):
            runtime = time.perf_counter() - start_time  # Calculate runtime
            return node, node_expanded_count, runtime
        explored.add(node.state)
        node_expanded_count += 1  # Increment for each node expanded

        for action in problem.actions(node.state):
            child_state = problem.result(node.state, action)
            if child_state not in explored:
                child_node = Node(child_state, node, action, node.path_cost + problem.path_cost(node.path_cost, node.state, action, child_state))
                if child_node not in frontier:
                    frontier.append((child_node.path_cost + heuristic(child_node), child_node))
                elif child_node in frontier and frontier.pop(child_node) > child_node.path_cost + heuristic(child_node):
                    frontier.remove(child_node)
                    frontier.append((child_node.path_cost + heuristic(child_node), child_node))

    runtime = time.perf_counter() - start_time  # Calculate runtime if no solution found
    return None, node_expanded_count, runtime  # Return runtime and count even if no solution

#_______________________________________________________________________________

import time
from collections import deque, namedtuple

def bfs_multi_goal(problem):
    """
    Performs a breadth-first search to find the shortest path that visits all specified goal states in a grid environment.

    Args:
        problem: An instance of the Problem class designed to handle multiple goals.

    Returns:
        list: The shortest path as a list of actions leading to a state where all goals have been visited, or None if no such path exists.
        int: Total number of nodes expanded.
        float: Total runtime of the search in seconds.
    """
    State = namedtuple('State', ['position', 'visited_goals'])
    initial_visited_goals = frozenset([problem.initial]) if problem.initial in problem.goal_states else frozenset()
    queue = deque([State(problem.initial, initial_visited_goals)])
    visited = set(queue)
    path_from_start = {State(problem.initial, initial_visited_goals): None}
    expanded_nodes = 0  # Node expansion counter
    start_time = time.perf_counter()  # Start the timer

    while queue:
        current_state = queue.popleft()
        current_position, visited_goals = current_state
        expanded_nodes += 1  # Increment for each node processed

        if visited_goals == frozenset(problem.goal_states):
            runtime = time.perf_counter() - start_time  # Calculate runtime
            return reconstruct_path(current_state, path_from_start), expanded_nodes, runtime

        for action in problem.actions(current_position):
            new_position = problem.result(current_position, action)
            new_visited_goals = visited_goals | ({new_position} if new_position in problem.goal_states else set())
            new_state = State(new_position, frozenset(new_visited_goals))

            if new_state not in visited:
                visited.add(new_state)
                queue.append(new_state)
                path_from_start[new_state] = (current_state, action)

    runtime = time.perf_counter() - start_time  # Calculate runtime if no solution found
    return None, expanded_nodes, runtime  # Return results with node count and runtime

def reconstruct_path(state, path_from_start):
    result_path = []
    while path_from_start[state] is not None:
        state, action = path_from_start[state]
        result_path.append(action)
    result_path.reverse()  # Path needs to be reversed to show from start to finish
    return result_path

#_______________________________________________________________________________
def read_grid_data(filename):
#Reading specified grid environment
    with open(filename, "r") as f:
        rows, cols = [int(x) for x in f.readline().strip("[]\n").split(",")]
        start_pos = tuple(int(x) for x in f.readline().strip("()\n").split(","))

        goal_states = []
        for line in f.readline().strip(" |").split(" | "):
            goal_states.append(tuple(int(x) for x in line.strip("()\n").split(",")))

        walls = []
        for line in f:
            x1, y1, width, height = [int(x) for x in line.strip("()\n").split(",")]
            walls.append((x1, y1, width, height))
        
        grid_data = {
        'rows': rows,
        'cols': cols,
        'start_pos': start_pos,
        'goal_states': goal_states,
        'walls': walls,
        }
    return grid_data

# RoboNav Problem

class RobotNavigationMain(Problem):
    """
    A specialized version of the Problem class for navigating a robot in a grid environment
    with obstacles and multiple goal states.
    """
    def __init__(self, initial, rows, cols, start_pos, walls, goal_states):
        """
        Initializes the navigation problem with grid specifics and obstacles.

        Args:
            initial (tuple): The initial position of the robot (x, y).
            rows (int): Number of rows in the grid.
            cols (int): Number of columns in the grid.
            start_pos (tuple): Starting position of the robot.
            walls (list of tuples): List of obstacles each defined by (x1, y1, width, height).
            goal_states (list of tuples): List of goal positions in the grid.
        """
        super().__init__(initial, goal_states)
        """ Define goal state and initialize a problem """
        self.rows = rows
        self.cols = cols
        self.start_pos = start_pos
        self.goal_states = goal_states
        self.walls = walls       

        # Initializing the grid based on  walls.
        self.grid = [[0 for _ in range(self.cols)] for _ in range(self.rows)]
        for x1, y1, width, height in self.walls:
            for y in range(y1, min(y1 + height, self.rows)):
                for x in range(x1, min(x1 + width, self.cols)):
                    self.grid[y][x] = 1

    def actions(self, state):
        """
        Determines the possible actions from a given state based on robot's position and obstacles.

        Args:
            state (tuple): The current state or position of the robot (col, row).

        Returns:
            list: List of possible actions ('UP', 'DOWN', 'LEFT', 'RIGHT') from the current state.
        """
        col, row = state

        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']

        if row == 0: 
            possible_actions.remove('UP')
        if col == 0:
            possible_actions.remove('LEFT')
        if row == self.rows - 1: 
            possible_actions.remove('DOWN')
        if col == self.cols - 1:
            possible_actions.remove('RIGHT')

        # Check for collisions with walls
        for action in possible_actions[:]:
            if self.get_new_pos(state, action) is None:
                possible_actions.remove(action)
            else:
                continue

        return possible_actions

    def get_new_pos(self, state, action):
        """
        Computes new position based on the current state and an action.

        Args:
            state (tuple): Current position (col, row) of the robot.
            action (str): Action to be taken from the current state.

        Returns:
            tuple or None: New position as a result of the action or None if action leads to an obstacle.
        """
        col, row = state
        x, y = col, row

        delta = {'UP': (0, -1), 'LEFT': (-1, 0), 'DOWN': (0, 1), 'RIGHT': (1, 0)}
        delta_col, delta_row = delta[action]
        new_y = y + delta_row
        new_x = x + delta_col

        if 0 <= new_y < self.rows and 0 <= new_x < self.cols and not self.is_wall(new_y, new_x):
            return new_x, new_y # Return new position if valid
        else:
            return None 
        
    def get_max_depth(self):
        return 10

    def result(self, state, action):
        """
        Given the state (robot's position) and action, return the new state.
        """
        col, row = state
        x, y = col, row
        new_state = tuple
        delta = {'UP': (0, -1), 'LEFT': (-1, 0), 'DOWN': (0, 1), 'RIGHT': (1, 0)}
        dcol, drow = delta[action]
        new_row, new_col = y + drow, x + dcol
        new_state = new_col, new_row
        return new_state
    
    def goal_test(self, state):
        """
        Check if the current state (robot's position) is a goal state.
        """
        return state in self.goal_states
    
    def is_wall(self, row, col):
        """
        Checks if the specified grid position contains a wall
        """
        return 0 <= row < self.rows and 0 <= col < self.cols and self.grid[row][col] == 1
    
    def heuristic(self, node):
        """
        Calculates a heuristic value for A* and greedy algorithms based on the current node.

        Args:
            node (Node): The current node for which to calculate the heuristic.

        Returns:
            int: Heuristic value estimating the cost from the current node to the closest goal
            Default heuristic function used is h(n) = number of misplaced tiles.
        """
        return sum(s != g for (s, g) in zip(node.state, self.goal))

# ______________________________________________________________________________

def main(filename, method):
    grid_data = read_grid_data(filename)
    problem = RobotNavigationMain(
        initial=grid_data['start_pos'],
        rows=grid_data['rows'],
        cols=grid_data['cols'],
        start_pos=grid_data['start_pos'],
        walls=grid_data['walls'],
        goal_states=grid_data['goal_states']
    )

    methods = {
        'GBFS': lambda: greedy_best_first_search(problem, lambda n: problem.heuristic(n)),
        'AS': lambda: astar_search(problem),
        'DFS': lambda: depth_first_search(problem),
        'BFS': lambda: breadth_first_search(problem),
        'CU1': lambda: iterative_deepening_search(problem),
        'CU2': lambda: directional_astar_search(problem),
        'MULTIBFS': lambda: bfs_multi_goal(problem)  # Add this line for multi-goal BFS
    }

    if method in methods:
        result, expanded_nodes, runtime = methods[method]()
        if result:
            if method == 'MULTIBFS':
                # Assuming MultiBFS returns a single path list directly
                print(f"{filename} {method}")
                print(f"goal reached with number_of_nodes {len(result)}")
                print(' '.join(result))
            else:
                # Handle single goal path results
                path = []
                node = result
                while node and node.parent:
                    path.insert(0, node.action)
                    node = node.parent
                print(f"{filename} {method}")
                print(f"goal {result.state} number_of_nodes {len(path)}")
                print(' '.join(str(p) for p in path if p is not None))
            print(f"Nodes expanded: {expanded_nodes}")
            print(f"Runtime: {runtime:.7f} seconds")
        else:
            print(f"{filename} {method}")
            print("No goal is reachable;",expanded_nodes)
    else:
        print(f"Error: Unknown method '{method}'. Valid methods are 'GBFS', 'AS', 'DFS', 'BFS', 'CU1', 'CU2', 'MULTIBFS'.")
        

if __name__ == "__main__":
    if len(sys.argv) == 3:
        grid_filename = sys.argv[1]
        search_method = sys.argv[2]
        main(grid_filename, search_method)
    else:
        print("Usage: python script.py <filename> <method>")
        print("Methods available: GBFS, AS, DFS, BFS, CU1, CU2, MULTIBFS")

