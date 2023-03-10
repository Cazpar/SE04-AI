"""
Author: Casper McGuire Stillinge
Description: This program solves a cleaning robot problem using the A* search algorithm.
The heuristic used in this code is the Manhattan distance.
"""
import heapq
from collections import namedtuple

# Define a named tuple to store information about a search node
Node = namedtuple('Node', ['state', 'parent', 'depth', 'heuristic', 'path_cost'])
# Override the __repr__ method of the Node named tuple to display custom information about the node when printed
Node.__repr__ = lambda self: f"State: {self.state} - Depth: {self.depth} - Path cost: {self.path_cost}"

# Calculate the heuristic value of a given state, which is the minimum number of moves required to clean all the dirty squares in the state
def calculate_heuristic(state: tuple) -> int:
    dirty_squares = [loc for loc, dirtiness in enumerate(state) if dirtiness == 'Dirty']
    # If there are no dirty squares, the heuristic is 0
    if not dirty_squares:
        return 0
    
    # Otherwise, calculate the minimum number of moves required to clean all dirty squares
    moves = []
    for square in dirty_squares:
        robot_pos = None
        for i, element in enumerate(state):
            if element in ['A', 'B']:
                robot_pos = i
                break
        moves.append(abs(robot_pos - square))  # Add the distance from the robot to the dirty square
    return min(moves)  # Return the minimum number of moves required to clean all dirty squares

def path(node):
    # Return a list of nodes from the start node to the given node
    path_list = [node]
    while node.parent:
        node = node.parent
        path_list.append(node)
    return list(reversed(path_list))

def tree_search(initial_state, goal_states):
    # Perform A* search algorithm
    fringe = []  # Initialize a priority queue to store the search nodes
    visited = set()  # Initialize a set to store the visited states
    heapq.heappush(fringe, Node(initial_state, None, 0, calculate_heuristic(initial_state), 0))  # Add the initial state to the priority queue

    while fringe:
        node = heapq.heappop(fringe)  # Pop the node with the lowest path cost from the priority queue

        if node.state in visited:  # If the state has already been visited, skip it
            continue

        visited.add(node.state)  # Add the state to the visited set

        if node.state in goal_states:  # If the current state is a goal state, return the solution path
            return path(node)

        for child_state, cost in successor_fn(node.state):  # For each successor state, calculate its cost and add it to the priority queue
            child_heuristic = calculate_heuristic(child_state)
            child_node = Node(child_state, node, node.depth + 1, child_heuristic, node.path_cost + cost)
            if child_node.state not in visited:
                heapq.heappush(fringe, child_node)

    return []  # If no solution is found, return an empty list

def successor_fn(state):
    # Return the possible successor states of a given state, along with their costs
    return STATE_SPACE[state]

# Define the initial state, goal states, and the state space
INITIAL_STATE = ('A', 'Dirty', 'Dirty')
GOAL_STATES = [('A', 'Clean', 'Clean'), ('B', 'Clean', 'Clean')]

STATE_SPACE = {
    ('A', 'Dirty', 'Dirty'): [(('A', 'Clean', 'Dirty'), 1), (('B', 'Dirty', 'Dirty'), 2)],
    ('B', 'Dirty', 'Dirty'): [(('B', 'Dirty', 'Clean'), 1)],

    ('B', 'Dirty', 'Clean'): [(('A', 'Dirty', 'Clean'), 2)],
    ('A', 'Dirty', 'Clean'): [(('A', 'Clean', 'Clean'), 1)],

    ('A', 'Clean', 'Dirty'): [(('B', 'Clean', 'Dirty'), 2)],
    ('B', 'Clean', 'Dirty'): [(('B', 'Clean', 'Clean'), 1)],

    ('A', 'Clean', 'Clean'): [],
    ('B', 'Clean', 'Clean'): []
}

def run():
    path = tree_search(INITIAL_STATE, GOAL_STATES)
    print('\nSolution path:')
    for i, node in enumerate(path):
        print(f"Step {i+1}: State {node.state} - Depth {node.depth} - Path cost {node.path_cost} - Heuristic {node.heuristic}")

if __name__ == '__main__':
    run()
