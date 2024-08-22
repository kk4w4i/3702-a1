import sys
from constants import *
from environment import *
from state import State
import heapq

"""
solution.py

This file is a template you should use to implement your solution.

You should implement 

COMP3702 2024 Assignment 1 Support Code
"""

class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter

    class StateNode:
        def __init__(self, env, state, parent=None, action=None, cost=0):
            """
            :param env: environment
            :param state: state belonging to this node
            :param parent: parent of this node
            :param action: FORWARD, REVERSE, SPIN_LEFT, SPIN_RIGHT
            :param cost: the cost of the step 
            """
            self.env = env
            self.state = state
            self.parent = parent
            self.action = action
            self.cost = cost

        def get_path(self):
            """
            :return: A list of actions
            """
            path = []
            current = self
            while current.parent is not None:
                path.append(current.action)
                current = current.parent
            path.reverse()
            return path
        
        def get_successors(self):
            """
            :return: A list of successors StateNodes
            """
            successors = []
            for act in BEE_ACTIONS:
                success, action_cost, next_state = self.env.perform_action(self.state, act)
                if success:
                    successors.append(Solver.StateNode(self.env, next_state, self, act, self.cost + action_cost))
            return successors

        def __lt__(self, other):
            return self.cost < other.cost

    # === Uniform Cost Search ==========================================================================================
    def solve_ucs(self, verbose=True):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of BEE_ACTIONS)
        """
        initial_state = self.environment.get_init_state()
        n_expanded = 0
        frontier = [self.StateNode(self.environment, initial_state)]
        heapq.heapify(frontier)

        visited = {initial_state: 0}
        
        while frontier:
            self.loop_counter.inc()
            n_expanded += 1
            node = heapq.heappop(frontier)

            if self.environment.is_solved(node.state):
                if verbose:
                    print(f'Visited Nodes: {len(visited.keys())},\t\tExpanded Nodes: {n_expanded},\t\t'
                        f'Nodes in Frontier: {len(frontier)}')
                    print(f'Cost of Path (with Costly Moves): {node.cost}')
                return node.get_path()
            
            successors = node.get_successors()
            for s in successors:
                if s.state not in visited or s.cost < visited[s.state]:
                    visited[s.state] = s.cost
                    heapq.heappush(frontier, s)

        return None

    # === A* Search ====================================================================================================

    def preprocess_heuristic(self):
        """
        Perform pre-processing (e.g. pre-computing repeatedly used values) necessary for your heuristic,
        """
        

    def compute_heuristic(self, state):
        """
        Compute a heuristic value h(n) for the given state.
        :param state: given state (GameState object)
        :return a real number h(n)
        """
        def hex_distance(row1, col1, row2, col2, memo):
            if (row1, col1, row2, col2) in memo:
                return memo[(row1, col1, row2, col2)]
            
            def to_cube(row, col):
                x = col - (row - (row & 1)) // 2
                z = row
                y = -x - z
                return x, y, z

            x1, y1, z1 = to_cube(row1, col1)
            x2, y2, z2 = to_cube(row2, col2)
            distance = (abs(x2 - x1) + abs(y2 - y1) + abs(z2 - z1)) // 2
            memo[(row1, col1, row2, col2)] = distance
            return distance
    
        uncovered_targets = 0
        total_distance = 0
        memo = {}

        total_distance = 0
        widget_cells = [
            widget_get_occupied_cells(self.environment.widget_types[i], state.widget_centres[i], state.widget_orients[i])
            for i in range(self.environment.n_widgets)
        ]
        for tgt in self.environment.target_list:
            min_distance = float('inf')
            target_covered = False
            for i in range(self.environment.n_widgets):
                if tgt in widget_cells[i]:
                    # Check if widget target is filled
                    target_covered = True
                    break
                for cell in widget_cells[i]:
                    # Manhattan distance between the widgets target and the unsolved widgets
                    distance = hex_distance(tgt[0], tgt[1], cell[0], cell[1], memo)
                    min_distance = min(min_distance, distance)

            if not target_covered:
                # Add to the heuristic
                uncovered_targets += 1
                total_distance += min_distance                

        """
        Heuristic:
        amount of uncovered targets
            + (
                aggregate distance between widgets current state and target state
                x 
                difference between pushing and pulling widgets to punish pushing as it costs more
            )
        """
        heuristic = uncovered_targets + ((total_distance) * (ACTION_PUSH_COST[FORWARD] - ACTION_PUSH_COST[REVERSE]))
        return heuristic

    def solve_a_star(self, verbose=True):
        """
        Find a path which solves the environment using A* search.
        :return: path (list of actions, where each action is an element of BEE_ACTIONS)
        """
        initial_state = self.environment.get_init_state()
        frontier = [(0 + self.compute_heuristic(initial_state), self.StateNode(self.environment, initial_state))]
        heapq.heapify(frontier)

        visited = {initial_state: 0}
        n_expanded = 0
        while len(frontier) > 0:
            self.loop_counter.inc()
            n_expanded += 1
            _, node = heapq.heappop(frontier)

            if self.environment.is_solved(node.state):
                if verbose:
                    print(f'Visited Nodes: {len(visited.keys())},\t\tExpanded Nodes: {n_expanded},\t\t'
                      f'Nodes in Frontier: {len(frontier)}')
                    print(f'Cost of Path (with Costly Moves): {node.cost}')
                return node.get_path()
            
            successors = node.get_successors()
            for s in successors:
                if s.state not in visited.keys() or s.cost < visited[s.state]:
                    visited[s.state] = s.cost
                    heapq.heappush(frontier, (s.cost + self.compute_heuristic(s.state), s))
        return None