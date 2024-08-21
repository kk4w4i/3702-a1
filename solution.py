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
    def solve_ucs(self):
        """
        Find a path which solves the environment using Uniform Cost Search (UCS).
        :return: path (list of actions, where each action is an element of BEE_ACTIONS)
        """
        initial_state = self.environment.get_init_state()
        frontier = [self.StateNode(self.environment, initial_state)]
        heapq.heapify(frontier)

        visited = {initial_state: 0}
        
        while frontier:
            self.loop_counter.inc()
            node = heapq.heappop(frontier)

            if self.environment.is_solved(node.state):
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
        pass

    def compute_heuristic(self, state):
        """
        Compute a heuristic value h(n) for the given state.
        :param state: given state (GameState object)
        :return a real number h(n)
        """
        pass

    def solve_a_star(self):
        """
        Find a path which solves the environment using A* search.
        :return: path (list of actions, where each action is an element of BEE_ACTIONS)
        """
        pass