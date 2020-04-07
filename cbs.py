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
    max_timestep = max(len(path1), len(path2))
    for t in range(max_timestep):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)
        if loc1 == loc2: # vertex
            return {'loc': [loc1], 'timestep': t}
        else: # edge
            past_loc1 = get_location(path1, t - 1)
            past_loc2 = get_location(path2, t - 1)
            if past_loc1 == loc2 and past_loc2 == loc1:
                return {'loc': [past_loc1, loc1], 'timestep': t}
    return None

def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    for i in range(len(paths) - 1):
        for j in range(i + 1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision is not None:
                collisions.append({'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep']})
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
    constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    constraint2 = {'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    if len(collision['loc']) != 1: # edge - reverse second constraint location list
        loc = collision['loc'][:]
        loc.reverse()
        constraint2 = {'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    return [constraint1, constraint2]

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
    agent = random.randint(0,1)
    if agent == 0:
        constraint1 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': True}
        constraint2 = {'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    else:
        loc = collision['loc'][:]
        loc.reverse()
        constraint1 = {'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': True}
        constraint2 = {'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    return [constraint1, constraint2]

def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

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

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
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
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['collisions'])

        # Task 3.2: Testing
        for collision in root['collisions']:
            print(standard_splitting(collision))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        # expanded_nodes = [] 
        while len(self.open_list) > 0:
            # print("Open list: {}".format(self.open_list))
            curr = self.pop_node()
            # expanded_nodes.append(curr)
            if len(curr['collisions']) == 0:
                # print("Expanded nodes list: {}".format(str(expanded_nodes)))
                self.print_results(curr)
                return curr['paths']
            else:
                collision = curr['collisions'][0]
                # print("In parent collisions, pick one collision {}".format(collision))
                # constraints = standard_splitting(collision)
                constraints = disjoint_splitting(collision)
                for constraint in constraints:
                    child_contraints = list(curr['constraints'])
                    child_contraints.append(constraint)
                    child_paths = list(curr['paths'])
                    agents_need_update = []
                    if constraint['positive']:
                        agents_need_update = paths_violate_constraint(constraint, child_paths)
                    else:
                        agents_need_update.append(constraint['agent'])
                    keep = True
                    for j in agents_need_update:
                        child_paths[j] = a_star(self.my_map, self.starts[j], self.goals[j], self.heuristics[j], j, child_contraints)
                        if child_paths[j] is None:
                            keep = False
                            break
                    if keep:
                        child = {'cost': get_sum_of_cost(child_paths),
                                'constraints': child_contraints,
                                'paths': child_paths,
                                'collisions': detect_collisions(child_paths)}
                        self.push_node(child)
        # print("Expanded nodes list: {}".format(str(expanded_nodes)))
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))