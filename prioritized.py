import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        max_path_len = 0
        map_size = sum(x.count(False) for x in self.my_map)
        upperbound = max_path_len + map_size
        # constraints.append({'agent': 0, 'loc': [(1, 5)], 'timestep': 4}) # 1.2
        # constraints.append({'agent': 1, 'loc': [(1, 2), (1,3)], 'timestep': 1}) # 1.3
        # constraints.append({'agent': 0, 'loc': [(1, 5)], 'timestep': 10}) # 1.4
        # 1.5
        # constraints.append({'agent': 1, 'loc': [(1, 2)], 'timestep': 2})
        # constraints.append({'agent': 1, 'loc': [(1, 3)], 'timestep': 2})
        # constraints.append({'agent': 1, 'loc': [(1, 4)], 'timestep': 2})

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            
            # Task 2.4
            if len(path) > upperbound:
                print("Reach upper bound {} for agent {} with path length {}".format(upperbound, i,len(path)))
                raise BaseException('No solutions')

            # Calculate an upper bound on the path length
            max_path_len = max(max_path_len, len(path))
            upperbound = max_path_len + map_size

            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            for agent in range(i + 1, self.num_of_agents + 1): # future agents
                for t in range(len(path)):
                    constraints.append({'agent': agent, 'loc': [path[t]], 'timestep': t})  # vertex
                    if t > 0:
                        constraints.append({'agent': agent, 'loc': [path[t], path[t-1]], 'timestep': t}) # edge
                        constraints.append({'agent': agent, 'loc': [path[t-1], path[t]], 'timestep': t}) # edge
           
                # Task 2.3 Additional Constraints (can also implemented by modify is_constrained function in single_agent_planner.py,then need to comment out it when choosing CBS)
                for t_future in range(len(path), upperbound):
                    constraints.append({'agent': agent, 'loc': [path[len(path) - 1]], 'timestep': t_future})
            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
