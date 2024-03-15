"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""
from single_agent_planner import *
class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    def __init__(self, my_map, start, goal, heuristics, agent_id):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.id = agent_id
        self.heuristics = heuristics
        self.priority = None
        # define as a tuple
        self.location = start
        self.constraints = []

    def path(self):
        self.current_path = a_star(self.my_map, self.location, self.goal, self.heuristics, self.id, self.constraints)

    def add_contraints(self, new_constraints):
        self.constraints.append(new_constraints)


    def replan(self):
        self.current_path = a_star(self.my_map, self.location, self.goal, self.heuristics, self.id, self.contraints)







