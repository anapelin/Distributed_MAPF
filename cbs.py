# import time as timer
# import heapq
# import random
# from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
#
#
# def detect_collision(path1, path2):
#     ##############################
#     # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
#     #           There are two types of collisions: vertex collision and edge collision.
#     #           A vertex collision occurs if both robots occupy the same location at the same timestep
#     #           An edge collision occurs if the robots swap their location at the same timestep.
#     #           You should use "get_location(path, t)" to get the location of a robot at time t.
#
#     pass
#
#
# def detect_collisions(paths):
#     ##############################
#     # Task 3.1: Return a list of first collisions between all robot pairs.
#     #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
#     #           causing the collision, and the timestep at which the collision occurred.
#     #           You should use your detect_collision function to find a collision between two robots.
#
#     pass
#
#
# def standard_splitting(collision):
#     ##############################
#     # Task 3.2: Return a list of (two) constraints to resolve the given collision
#     #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
#     #                            specified timestep, and the second constraint prevents the second agent to be at the
#     #                            specified location at the specified timestep.
#     #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
#     #                          specified timestep, and the second constraint prevents the second agent to traverse the
#     #                          specified edge at the specified timestep
#
#     pass
#
#
# def disjoint_splitting(collision):
#     ##############################
#     # Task 4.1: Return a list of (two) constraints to resolve the given collision
#     #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
#     #                            specified timestep, and the second constraint prevents the same agent to be at the
#     #                            same location at the timestep.
#     #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
#     #                          specified timestep, and the second constraint prevents the same agent to traverse the
#     #                          specified edge at the specified timestep
#     #           Choose the agent randomly
#
#     pass
#
#
# class CBSSolver(object):
#     """The high-level search of CBS."""
#
#     def __init__(self, my_map, starts, goals):
#         """my_map   - list of lists specifying obstacle positions
#         starts      - [(x1, y1), (x2, y2), ...] list of start locations
#         goals       - [(x1, y1), (x2, y2), ...] list of goal locations
#         """
#
#         self.my_map = my_map
#         self.starts = starts
#         self.goals = goals
#         self.num_of_agents = len(goals)
#
#         self.num_of_generated = 0
#         self.num_of_expanded = 0
#         self.CPU_time = 0
#
#         self.open_list = []
#
#         # compute heuristics for the low-level search
#         self.heuristics = []
#         for goal in self.goals:
#             self.heuristics.append(compute_heuristics(my_map, goal))
#
#     def push_node(self, node):
#         heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
#         print("Generate node {}".format(self.num_of_generated))
#         self.num_of_generated += 1
#
#     def pop_node(self):
#         _, _, id, node = heapq.heappop(self.open_list)
#         print("Expand node {}".format(id))
#         self.num_of_expanded += 1
#         return node
#
#     def find_solution(self, disjoint=True):
#         """ Finds paths for all agents from their start locations to their goal locations
#
#         disjoint    - use disjoint splitting or not
#         """
#
#         self.start_time = timer.time()
#
#         # Generate the root node
#         # constraints   - list of constraints
#         # paths         - list of paths, one for each agent
#         #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
#         # collisions     - list of collisions in paths
#         root = {'cost': 0,
#                 'constraints': [],
#                 'paths': [],
#                 'collisions': []}
#         for i in range(self.num_of_agents):  # Find initial path for each agent
#             path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
#                           i, root['constraints'])
#             if path is None:
#                 raise BaseException('No solutions')
#             root['paths'].append(path)
#
#         root['cost'] = get_sum_of_cost(root['paths'])
#         root['collisions'] = detect_collisions(root['paths'])
#         self.push_node(root)
#
#         # Task 3.1: Testing
#         print(root['collisions'])
#
#         # Task 3.2: Testing
#         for collision in root['collisions']:
#             print(standard_splitting(collision))
#
#         ##############################
#         # Task 3.3: High-Level Search
#         #           Repeat the following as long as the open list is not empty:
#         #             1. Get the next node from the open list (you can use self.pop_node()
#         #             2. If this node has no collision, return solution
#         #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
#         #                standard_splitting function). Add a new child node to your open list for each constraint
#         #           Ensure to create a copy of any objects that your child nodes might inherit
#
#         self.print_results(root)
#         return root['paths']
#
#
#     def print_results(self, node):
#         print("\n Found a solution! \n")
#         CPU_time = timer.time() - self.start_time
#         print("CPU time (s):    {:.2f}".format(CPU_time))
#         print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
#         print("Expanded nodes:  {}".format(self.num_of_expanded))
#         print("Generated nodes: {}".format(self.num_of_generated))


import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    max_time = max(len(path1), len(path2))
    for time in range(max_time):
        pos1 = get_location(path1, time)
        pos2 = get_location(path2, time)
        pos1p = get_location(path1, time + 1)
        pos2p = get_location(path2, time + 1)

        # Detect vertex collision
        if pos1 == pos2:
            return pos1, time

        # Detect edge collision
        if pos1 == pos2p and pos2 == pos1p:
            return (pos1, pos2), time + 1

    return None

8
def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = list()
    for idx1, agent1 in enumerate(paths):
        for idx2 in range(idx1 + 1, len(paths)):
            collision = detect_collision(agent1, paths[idx2])
            if collision:
                collisions.append({'a1': idx1, 'a2': idx2, 'loc': collision[0], 'timestep': collision[1]})
    return collisions


def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    constraints_collision = list()
    if isinstance(collision['loc'][0], int) and isinstance(collision['loc'][1], int):
        # Vertex collision
        constraints_collision.append(
            {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep']})
        constraints_collision.append(
            {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep']})
    elif isinstance(collision['loc'][0], tuple) and isinstance(collision['loc'][1], tuple):
        # Edge collision
        constraints_collision.append(
            {'agent': collision['a1'], 'loc': collision['loc'][1], 'timestep': collision['timestep']})
        constraints_collision.append(
            {'agent': collision['a2'], 'loc': collision['loc'][0], 'timestep': collision['timestep']})
    else:
        raise BaseException('Wrong loc format collision detection')

    return constraints_collision


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    pass


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.sum_of_cost = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

        self.goal_reached = dict()

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, verbose=True, max_iter=200):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}

        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'], dict())
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)
            self.goal_reached[i] = (len(path), self.goals[i])

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        # print(root['collisions'])

        # Task 3.2: Testing
        # for collision in root['collisions']:
        #    print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        # curr = {'timestep': 0}
        iteration = 0
        while len(self.open_list) > 0:  # and not curr['timestep'] > max_simulation_time:
            curr = self.pop_node()

            iteration = iteration + 1
            if iteration > max_iter:
                raise BaseException("Max iterations exceeded!")

            if verbose:
                print('Iter:', iteration)

            if len(curr['collisions']) == 0:
                break

            # collision = random.choice(curr['collisions'])
            collision = curr['collisions'][0]
            constraints = standard_splitting(collision)

            for constraint in constraints:
                new_node = {'cost': 0,
                            'constraints': curr['constraints'].copy() + list([constraint]),
                            'paths': curr['paths'].copy(),
                            'collisions': []}
                a = constraint['agent']
                path = a_star(self.my_map, self.starts[a], self.goals[a], self.heuristics[a],
                              a, new_node['constraints'], self.goal_reached)
                if path is None:
                    # raise BaseException('No solutions')
                    continue
                if len(path) != 0:
                    new_node['paths'][a] = path
                    new_node['collisions'] = detect_collisions(new_node['paths'])
                    new_node['cost'] = get_sum_of_cost(new_node['paths'])
                    self.push_node(new_node)
                self.goal_reached[a] = (len(path), self.goals[a])

        self.CPU_time = timer.time() - self.start_time
        self.sum_of_cost = get_sum_of_cost(curr['paths'])

        if verbose:
            self.print_results(curr)

        return curr['paths']

    def print_results(self):
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(self.sum_of_cost))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
