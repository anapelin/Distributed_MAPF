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
        self.no_conflicts = 0   # number of conflicts had so far
        self.path()

    def path(self):
        """
        Performs A* and creates an
        array with all the steps taken by the agent:
        each position is a tuple with the location at time t.
        """
        self.current_path = a_star(self.my_map, self.location, self.goal, self.heuristics, self.id, self.constraints)

    def direction_path(self):
        """
        Vector of directions in the path consists of "right", "left","down", "up", "stays"
        direction is based on current location and next location
        """
        array = []
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        for i, step in enumerate(self.current_path):
            difference_y = self.current_path[i+1][0] - self.current_path[i][0]
            difference_y = self.current_path[i+1][1] - self.current_path[i][1]
            direction = (difference_y, difference_y)
            if direction == (0, -1):
                array.append("left")
            if direction == (1, 0):
                array.append("down")
            if direction == (0, 1):
                array.append("right")
            if direction == (-1, 0):
                array.append("up")
            if direction == (0, 0):
                array.append("stay")
        self.direction = array

    def direction_consistency(self):
        """
        Check whether agent is currently following a straight trajectory
        """
        pass



    def add_contraints(self, new_constraints):
        self.constraints.append(new_constraints)


    def replan(self):
        self.current_path = a_star(self.my_map, self.location, self.goal, self.heuristics, self.id, self.contraints)







