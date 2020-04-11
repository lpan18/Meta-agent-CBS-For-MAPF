import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from cbs import detect_collision, standard_splitting, disjoint_splitting, paths_violate_constraint
import copy

# collisions format {(a1, a2):[{'loc': [], 'timestep': t},{'loc': [], 'timestep': t}...] ...}
def detect_collisions(paths):
    collisions = {}
    for i in range(len(paths) - 1):
        for j in range(i + 1, len(paths)):
            collision = detect_collision(paths[i], paths[j])
            if collision is not None:
                if (i, j) not in collisions:
                    collisions[(i,j)] = []
                collisions[(i, j)].append(collision)
    return collisions

"""Count number of collision between two meta-agents"""
def count_collision(collisions, group1, group2):
    count = 0
    for i in group1:
        for j in group2:
            if i != j:
                count += len(collisions[(i,j)])
    return count

"""Check if two meta agents need to be merged, if yes, merge them"""
def merge_agents(node, B):
    groups = node['groups']
    collisions = node['collisions']
    max_conflicts = 0; max_i = 0; max_j = 0
    for i, group1 in enumerate(groups):
        for j, group2 in enumerate(groups):
            count = count_collision(collisions, group1, group2)
            if(count > max_conflicts):
                max_conflicts = count
                max_i = i
                max_j = j
    if conflict_matrix[max_i,max_j] > B:
        print("Merging meta-agents {} and {} with conflict count {}".format(groups[max_i], groups[max_j], max_conflicts))
        groups = copy.deepcopy(groups)
        groups[max_i] = groups[max_i] + groups[max_j]
        groups.pop(j)
        node['groups'] = groups
        return groups, max_i
    return groups, -1

class MetaAgentCBSSolver(object):
    """The high-level search of meta-agent CBS."""

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

        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        # Merging bound parameter
        self.B = 5

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
        # collisions    - dict of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': {},
                'groups': []} # each group is a meta-agent [[],[]...]
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        # print(root['collisions'])
        # for item in root['collisions'].items():
        #     collision = {'a1': item[0][0], 'a2': item[0][1], 'loc': item[1][0]['loc'], 'timestep': item[1][0]['timestep']}
        #     print(standard_splitting(collision))

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
            if len(curr['collisions']) == 0:
                self.print_results(curr)
                return curr['paths']
            item = next(iter(curr['collisions'].items())) # {(1, 2): [{'loc': [3, 4], 'timestep': 0}]}
            collision = {'a1': item[0][0], 'a2': item[0][1], 'loc': item[1][0]['loc'], 'timestep': item[1][0]['timestep']}
            # MA-CBS only
            # check if a new meta-agent needs to be formed
            groups, group_idx = merge_agents(node, self.B)
            if group_idx > 0: # should-merge
                # new_node = initialize_child_search_node(node)
                # low_level_search!(solver, mapf, new_node, [group_idx])
                # for agent_id in node.groups[group_idx]
                #     detect_conflicts!(new_node.conflict_table,new_node.solution,agent_id)
                # TODO update constraints, solution
                child = {}
                self.push_node(child)
                continue
            # basic CBS
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
                            'collisions': detect_collisions(child_paths),
                            'groups': None}
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