import time as timer
import heapq
import random
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from cbs import detect_collision, paths_violate_constraint
from low_level import constrained_od_mstar
from helper import convert_cons, convert_path

# if True - > use EPEA* instead of A* or A* + OD
epeastar=True
# if True -> use A* instead of A* + OD
astar=False

"""detect collision between all groups"""
def detect_collisions(paths, groups):
    for i in range(len(groups)-1):
        for j in range(i+1, len(groups)):
            new_collision = detect_collision_two_group(paths, groups, i, j)
            if new_collision is not None:
                return new_collision
    return None

"""detect collision between two groups eg. [3,4,5], [7,9]"""
def detect_collision_two_group(paths, groups, i, j):
    for agent1 in groups[i]:
        for agent2 in groups[j]:
            collision = detect_collision(paths[agent1], paths[agent2]) # paths need to be in order of agents
            if collision is not None:
                # save conflict group index
                return {'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep']}
    return None

def standard_splitting(collision):
    # 'group': the group it belongs to , 'agent': the agents that have the constraint
    constraint1 = {'group': collision['a1'], 'agent': collision['a1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    constraint2 = {'group': collision['a2'], 'agent': collision['a2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    if len(collision['loc']) != 1: # edge - reverse second constraint location list
        loc = collision['loc'][:]
        loc.reverse()
        constraint2 = {'group': collision['a2'], 'agent': collision['a2'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
    return [constraint1, constraint2]

"""Count number of collision between two meta-agents"""
def count_collision(conflict_matrix, group1, group2):
    count = 0
    for i in group1:
        for j in group2:
            conflict_matrix[i][j] += 1
            count += conflict_matrix[i][j]
            count += conflict_matrix[j][i]
    return count

"""Merge constraints"""
def update_constraints(idx1, idx2, child):
    # external constraints
    for constraint in child['constraints']:
        if constraint['group'] == idx1 or constraint['group'] == idx2:
            constraint['group'] = idx1
    # # internal constraints
    # collisions = child['collisions']

"""Generate a new child"""
def init_node_from_parent(curr):
    return {'cost': curr['cost'],
            'constraints': list(curr['constraints']),
            'paths': list(curr['paths']),
            'collisions': list(curr['collisions']),
            'groups': list(curr['groups'])}

"""Low-level search"""
def compute_paths(map, starts, goals, heuristics, meta_agent, child):
    if len(meta_agent) == 1:
        agent = meta_agent[0]
        path = a_star(map, starts[agent], goals[agent], heuristics[agent], agent, child['constraints'])
        if path is None:
            return False
        child['paths'][agent] = path
    else:
        t_starts = tuple(starts[i] for i in meta_agent)
        t_goals = tuple(goals[i] for i in meta_agent)
        planner = constrained_od_mstar.Constrained_Od_Mstar(map, t_starts, t_goals, convert_cons(child), epeastar=epeastar, astar=astar)
        path = planner.find_path_time_pad(t_starts)
        path = convert_path(path)
        j = 0
        for i in meta_agent:
            child['paths'][i] = path[j]
            j += 1
    return True

class MetaAgentCBSSolver(object):
    """The high-level search of meta-agent CBS."""

    def __init__(self, my_map, starts, goals, merge_thresh):
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

        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        # Merging bound parameter
        self.B = merge_thresh
        # Conflict count table
        self.conflict_matrix = [[0 for i in range(self.num_of_agents)] for j in range(self.num_of_agents)]

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
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
        init_groups = list([i, ] for i in range(self.num_of_agents))
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'groups': init_groups}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'], root['groups'])
        self.push_node(root)

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        while len(self.open_list) > 0:
            curr = self.pop_node()
            new_collision = detect_collisions(curr['paths'], curr['groups'])
            if new_collision is None:
                self.print_results(curr)
                print(curr['paths'])
                return curr['paths']
            new_constraints = standard_splitting(new_collision)
            # check should merge
            group_idx1 = new_collision['a1']
            group_idx2 = new_collision['a2']
            group1 = curr['groups'][group_idx1]
            group2 = curr['groups'][group_idx2]
            cnt = count_collision(self.conflict_matrix, group1, group2)
            if cnt > self.B: # should-merge
                print("Merging meta-agents {} and {} with total conflict count {}".format(group1, group2, cnt))
                child = init_node_from_parent(curr)
                # update constraints
                update_constraints(group_idx1, group_idx2, child)
                new_group = sorted(group1 + group2)
                child['groups'][group_idx1] = new_group
                child['groups'].pop(group_idx2)
                # update solutions
                agents_need_update = []
                agents_need_update.append(child['groups'][group_idx1])
                keep = True
                # print(agents_need_update)
                for meta_agent in agents_need_update:
                    keep = compute_paths(self.my_map, self.starts, self.goals, self.heuristics, meta_agent, child)
                    if not keep:
                        break
                if keep:
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)
                continue            
            else: # basic CBS
                for constraint in new_constraints:
                    child = init_node_from_parent(curr)
                    child['constraints'].append(constraint)
                    child['collisions'].append(new_collision)
                    agents_need_update = [] #[[],[]]
                    if constraint['positive']:
                        agents_need_update = paths_violate_constraint(constraint, child['paths'])
                    else:
                        agents_need_update.append(child['groups'][constraint['group']])
                    keep = True
                    for meta_agent in agents_need_update:
                        keep = compute_paths(self.my_map, self.starts, self.goals, self.heuristics, meta_agent, child)
                        if not keep:
                            break
                    if keep:
                        child['cost'] = get_sum_of_cost(child['paths'])
                        self.push_node(child)
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.3f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))