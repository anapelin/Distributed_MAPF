"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer

import numpy as np

from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions

class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        # T.B.D.

        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))  # each goal has own set of heuristics, appended to list -> ordered by agent index

    def detect_warning(self, location1, location2):

        # compute distance between agents
        radius = np.sqrt((location1[0]-location2[0])**2 + (location1[1] - location2[1]) **2) #float element
        if radius < 3:
            return True
        return False

    def direction_priority(self, direction1, direction2):
        """
        Fill in the direction rule right > left > up > down > stays
        """
        pass

    def solve_priority(self, agent1: AircraftDistributed, agent2: AircraftDistributed):
        return min(agent1.id, agent2.id), return min(agent1.id, agent2.id)


    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations. 
        
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        start_time = timer.time()
        result_dict = {}
        result = []
        self.CPU_time = timer.time() - start_time
        longest_path = -1

        # create a dictionary of agents
        DAgent = {}

        # Create agent objects with AircraftDistributed class
        for i in range(self.num_of_agents):
            DAgent[f'Agent{i}'] = AircraftDistributed(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i)
            DAgent[f'Agent{i}'].path()
            result_dict[f'Agent {i}'] = DAgent[f'Agent{i}'].current_path
            result.append(DAgent.current_path)

            # keep track of how long the longest path planned is
            longest_path = max(len(DAgent.current_path), longest_path)

        agents_at_goals = []
        # Create the time loop
        for time in range(longest_path):
            # stop the loop when the system is solved
            if len(agents_at_goals) == len(self.num_of_agents):
                print("It is solved!!")
                break
            # check whether future conflicts
            for i in range(len(self.num_of_agents - 1)):
                for j in range(len(self.num_of_agents)):
                    if self.detect_warning(DAgent[f'Agent{i}'].location, DAgent[f'Agent{j}'].location):
                        print("They are too close - they will collide")

                        priority_agent = self.solve_priority(DAgent[f'Agent{i}'],  DAgent[f'Agent{j}'])[0]
                        non_priority_agent = self.solve_priority(DAgent[f'Agent{i}'],  DAgent[f'Agent{j}'])[1]

                        next_steps_priority_agent = DAgent[f'Agent{priority_agent}'].current_path[time:(time + 3)]

                        non_prioritized_constraints = []

                        for i, step in enumerate(next_steps_priority_agent):
                            # make constraints
                            v_constraint = {'agent': priority_agent, 'loc': [step], 'timestep': time, 'positive': False}
                            e_constraint = {'agent': priority_agent, 'loc': [step, DAgent[f'Agent{priority_agent}'].location], 'timestep': time, 'positive': False}


                        #replan

                        pass


        
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)

        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)
