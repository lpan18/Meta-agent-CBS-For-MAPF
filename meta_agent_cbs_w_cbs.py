import time as timer
import heapq
import random
import copy
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from cbs import detect_collision, paths_violate_constraint, CBSSolver

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
                return {'group1': i, 'group2': j, 'a1': groups[i], 'a2': groups[j], 'loc': collision['loc'], 'timestep': collision['timestep']}
    return None

def standard_splitting(collision):
    # 'group': the group it belongs to , 'agent': the agents that have the constraint
    constraint1 = {'group': collision['group1'], 'agent': collision['a1'], 'cause': collision['group2'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    constraint2 = {'group': collision['group2'], 'agent': collision['a2'], 'cause': collision['group1'], 'loc': collision['loc'], 'timestep': collision['timestep'], 'positive': False}
    if len(collision['loc']) != 1: # edge - reverse second constraint location list
        loc = collision['loc'][:]
        loc.reverse()
        constraint2 = {'group': collision['group2'], 'agent': collision['a2'], 'cause': collision['group1'], 'loc': loc, 'timestep': collision['timestep'], 'positive': False}
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
def update_constraints(group1, group2, group_idx1, group_idx2, child):
    # loop over all constraints to update them
    internal_idx = []
    for (idx, constraint) in enumerate(child['constraints']):
        # if the group equals to group 1
        if constraint['group'] == group_idx1:
            # if the group equals to group 2, it's an internal constraint
            if constraint['cause'] == group_idx2:
                # internal constraint should be removed, save its idx first
                internal_idx.append(idx)
            # for other cause leave it be
        elif constraint['group'] == group_idx2:
            # change group to group 1
            child['constraints'][idx]['group'] = group_idx1
            # if the group equals to group 2, it's an internal constriant
            if constraint['cause'] == group_idx1:
                # internal constraint should be removed, save its idx first
                internal_idx.append(idx)
        elif constraint['cause'] == group_idx2:
            child['constraints'][idx]['cause'] = group_idx1
    # remove the internal constraints
    internal_idx = sorted(internal_idx, reverse=True)
    for internal_idx_ in internal_idx:
        child['constraints'].pop(internal_idx_)
    new_group = sorted(group1 + group2)
    child['groups'][group_idx1] = new_group
    # do not remove group 2 after updating constrants
    # or else the group idx in constraints would be influenced
    # just make it empty list
    child['groups'][group_idx2] = []

"""Generate a new child"""
def init_node_from_parent(curr):
    return {'cost': curr['cost'],
            'constraints': copy.deepcopy(curr['constraints']),
            'paths': list(curr['paths']),
            'groups': list(curr['groups'])}

def flatten_constraints(constraints):
    flat_constraints = []
    for constraint in constraints:
        for agent in constraint['agent']:
            flat_constraints.append(copy.copy(constraint))
            flat_constraints[len(flat_constraints)-1]['agent'] = agent
    return flat_constraints

"""Low-level search"""
def compute_paths(map, starts, goals, heuristics, agents_need_update, child, group_idx=None):
    if len(agents_need_update) == 1:
        agent = agents_need_update[0]
        path = a_star(map, starts[agent], goals[agent], heuristics[agent], agent, flatten_constraints(child['constraints']))
        if path is None:
            return False
        child['paths'][agent] = path
    else:
        cbs = CBSSolver(map, [starts[m] for m in agents_need_update], [goals[m] for m in agents_need_update])
        agents_mapping = {}
        for agent in agents_need_update:
            agents_mapping[agent] = len(agents_mapping)
        meta_constraints = []
        for constraint in child['constraints']:
            if constraint['group'] == group_idx:
                for agent in constraint['agent']:
                    meta_constraints.append(copy.copy(constraint))
                    # print('agents:', constraint['agent'])
                    # print('mapping:', agents_mapping)
                    meta_constraints[len(meta_constraints)-1]['agent'] = agents_mapping[agent]
        path = cbs.find_solution(meta_constraints=meta_constraints)
        if path is None:
            return False
        for (idx, m) in enumerate(agents_need_update):
            child['paths'][m] = path[idx]
        # print("path", path)
    return True

class MetaAgCBSWithCBS(object):
    """The high-level search of meta-agent CBS with CBS as low level solver."""

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
        heapq.heappush(self.open_list, (node['cost'], self.num_of_generated, node))
        # print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, id, node = heapq.heappop(self.open_list)
        # print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations
        """

        self.start_time = timer.time()
        init_groups = list([i, ] for i in range(self.num_of_agents))
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'groups': init_groups}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        self.push_node(root)

        # High-Level Search
        while len(self.open_list) > 0:
            if timer.time() - self.start_time  > 100:
                raise Exception('timeout|'+str(self.num_of_expanded)+'|'+str(self.num_of_generated)+'|'+str(round(timer.time()-self.start_time, 2)))
            # print("===high-level loop===")
            curr = self.pop_node()
            # print("curr", curr)
            new_collision = detect_collisions(curr['paths'], curr['groups'])
            if new_collision is None:
                # print(curr['paths'])
                self.print_results(curr)
                CPU_time = timer.time() - self.start_time
                return (curr['paths'], "{:.03f}".format(CPU_time), self.num_of_expanded, self.num_of_generated)
            # check should merge
            group_idx1 = new_collision['group1']
            group_idx2 = new_collision['group2']
            group1 = curr['groups'][group_idx1]
            group2 = curr['groups'][group_idx2]
            cnt = count_collision(self.conflict_matrix, group1, group2)
            if cnt > self.B: # should-merge
                # print("Merging meta-agents {} and {} with total conflict count {}".format(group1, group2, cnt))
                child = init_node_from_parent(curr)
                # update constraints
                update_constraints(group1, group2, group_idx1, group_idx2, child)
                # update solutions
                agents_need_update = child['groups'][group_idx1]
                keep = compute_paths(self.my_map, self.starts, self.goals, self.heuristics, agents_need_update, child, group_idx1)
                # print("keep", keep)
                if keep:
                    child['cost'] = get_sum_of_cost(child['paths'])
                    self.push_node(child)
                    # print("openlist", self.open_list)
                continue
            else: # basic CBS
                new_constraints = standard_splitting(new_collision)
                for constraint in new_constraints:
                    child = init_node_from_parent(curr)
                    child['constraints'].append(constraint)
                    agents_need_update = constraint['agent']
                    keep = compute_paths(self.my_map, self.starts, self.goals, self.heuristics, agents_need_update, child)
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