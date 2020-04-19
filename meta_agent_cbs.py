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
    # collision_flag = False
    # for agent1 in groups[i]:
    #     for agent2 in groups[j]:
    #         collision = detect_collision(paths[agent1], paths[agent2]) # paths need to be in order of agents
    #         if collision is not None:
    #             collision_flag = True
    # if collision_flag != False:
    #     # for every pair of agents in two groups, create a collision
    #     for agent1 in groups[i]:
    #         for agent2 in groups[j]:
    #             collisions = []
    #             collisions.append({'group1': i, 'group2': j, 'a1': agent1, 'a2': agent2, 'loc': collision['loc'], 'timestep': collision['timestep']})
    #     return collisions
    # else:
    #     return None
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
def update_constraints(new_group, idx1, idx2, child):
    # external constraints
    group1 = child['groups'][idx1]
    group2 = child['groups'][idx2]
    for constraint in child['constraints']:
        if constraint['group'] == group1 or constraint['group'] == group2:
            constraint['group'] == new_group
    # # internal constraints
    # collisions = child['collisions']

"""Generate a new child"""
def init_node_from_parent(curr):
    return {'cost': curr['cost'],
            'constraints': list(curr['constraints']),
            'paths': list(curr['paths']),
            'collisions': list(curr['collisions']),
            'groups': list(curr['groups'])}

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
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def flatten_constraints(self, constraints):
        flat_constraints = []
        for constraint in constraints:
            for agent in constraint['agent']:
                flat_constraints.append(copy.copy(constraint))
                flat_constraints[len(flat_constraints)-1]['agent'] = agent
        return flat_constraints

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations
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
        collisions_ = detect_collisions(root['paths'], root['groups'])
        if collisions_ is None:
            return root['paths']
        root['collisions'].append(collisions_)
        self.push_node(root)

        # High level search
        while len(self.open_list) > 0:
            curr = self.pop_node()
            new_collision = detect_collisions(curr['paths'], curr['groups'])
            if new_collision is None:
                self.print_results(curr)
                return curr['paths']
            
            # check should merge
            group_idx1 = new_collision['group1']
            group_idx2 = new_collision['group2']
            group1 = curr['groups'][group_idx1]
            group2 = curr['groups'][group_idx2]
            cnt = count_collision(self.conflict_matrix, group1, group2)
            if cnt > self.B: # should-merge
                print("Merging meta-agents {} and {} with total conflict count {}".format(group1, group2, cnt))
                child = init_node_from_parent(curr)
                # update constraints
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

                # update solutions
                child_paths = child['paths']
                agents_need_update = child['groups'][group_idx1]
                keep = True
                print(agents_need_update)
                # use cbs with disjoint as low level search algorithm (just for testing use)
                cbs = CBSSolver(self.my_map, [self.starts[m] for m in agents_need_update], [self.goals[m] for m in agents_need_update])
                agents_mapping = {}
                for agent in agents_need_update:
                    agents_mapping[agent] = len(agents_mapping)
                meta_constraints = []
                print('cccccc----', child['constraints'])
                if len([self.starts[m] for m in agents_need_update]) > 2:
                    exit(0)
                for constraint in child['constraints']:
                    if constraint['group'] == group_idx1:
                        for agent in constraint['agent']:
                            meta_constraints.append(copy.copy(constraint))
                            meta_constraints[len(meta_constraints)-1]['agent'] = agents_mapping[agent]
                path_ = cbs.find_solution(meta_constraints=meta_constraints)
                if path_ != None:
                    # update paths
                    for (idx, m) in enumerate(agents_need_update):
                        child['paths'][m] = copy.copy(path_[idx])
                    child['cost'] = get_sum_of_cost(child_paths)
                    self.push_node(child)
                continue
            else: # basic CBS
                new_constraints = standard_splitting(new_collision)
                for constraint in new_constraints:
                    child = init_node_from_parent(curr)
                    child['constraints'].append(constraint)
                    child['collisions'].append(new_collision)
                    child_paths = child['paths']
                    agents_need_update = constraint['agent']
                    keep = True
                    for j in agents_need_update:
                        child_paths[j] = a_star(self.my_map, self.starts[j], self.goals[j], self.heuristics[j], j, self.flatten_constraints(child['constraints']))
                        if child_paths[j] is None:
                            keep = False
                            break
                    if keep:
                        child['cost'] = get_sum_of_cost(child_paths)
                        self.push_node(child)
        self.print_results(root)
        return root['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))