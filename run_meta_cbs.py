#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from meta_agent_cbs_w_cbs import MetaAgCBSWithCBS
from meta_agent_cbs_w_mstar import MetaAgCBSWithMstar
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import random
from os import listdir
from os.path import isfile, join

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    print(str(len(my_map)) + ',' + str(len(my_map[0])))
    print(locations)
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(map_file, scen_file, agents_limit = None):
    ''' load map info
    '''
    map_f = Path(map_file)
    if not map_f.is_file():
        raise BaseException(map_file + " does not exist.")
    map_f = open(map_file, 'r')

    # line 1: type
    line = map_f.readline()
    
    # line 2: rows number / height
    line = map_f.readline()
    tmp = line.split(' ')
    if tmp[0] != 'height':
        raise BaseException("Error loading rows number")
    rows = int(tmp[1])
    
    # line 3: cols number / width
    line = map_f.readline()
    tmp = line.split(' ')
    if tmp[0] != 'width':
        raise BaseException("Error loading cols number")
    cols = int(tmp[1])

    # line 4: starting signal of maps
    line = map_f.readline()
    if line.split()[0] != 'map':
        raise BaseException("Error loading map signal")

    my_map = []
    for _ in range(rows):
        line = map_f.readline()
        line = line.strip()
        if len(line) != cols:
            raise BaseException("Inconsistent map cell number {0} and row number {1}".format(len(line), cols))
        my_map.append([])
        for cell in line:
            if cell == '@' or cell == 'T':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
            else:
                raise BaseException("Unknown map cell: " + cell + "\n" + line)
    
    ''' load scenario info
    '''
    scen_f = Path(scen_file)
    if not scen_f.is_file():
        raise BaseException(scen_file + " does not exist.")
    scen_f = open(scen_file, 'r')

    # line 1: version number
    line = scen_f.readline()
    
    # load start and goal positions
    starts_all = []
    goals_all = []
    while True:
        line = scen_f.readline()
        if not line:
            break
        tmp = line.split()
        starts_all.append((int(tmp[5]), int(tmp[4])))
        goals_all.append((int(tmp[7]), int(tmp[6])))

    if len(starts_all) < agents_limit:
        print("number of agents in scen smaller than agents_limit")
        return my_map, starts_all, goals_all
    
    # randomly choose starts and goals
    idx_list = list(range(len(starts_all)))
    chosen_idx = random.choices(idx_list, k=agents_limit)
    starts = [starts_all[m] for m in chosen_idx]
    goals = [goals_all[m] for m in chosen_idx]
    
    return my_map, starts, goals 


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('-s', dest='solver', type=str, default="all",
                        help='The solver to use (CBS, MetaCBSwCBS, MetaCBSwMstar), defaults to run all solvers')
    parser.add_argument('-b', dest='merge_thresh', type=int, default=5,
                        help='Merge threshold for Meta-CBS')
    parser.add_argument('-a', dest='animation',  default=False,
                        help='Animation')

    args = parser.parse_args()

    result_file = open("results.csv", "a", buffering=1)
    result_file.write("===================\n")
    result_file.write("solver, cost, time, n_exp, n_gen, map_file, scen_file, agents_limit, conflict_bound, success \n")

    # config the maps and scenarios path
    # map_path = 'more/maps/random-32-32-10.map'
    # scen_path = 'more/scenarios/random-32-32-10-even-1.scen'

    # map_path = 'more/maps/den312d.map'
    # scen_path = 'more/scenarios/den312d-even-1.scen'

    map_folder = 'more/maps/3/'
    scen_folder = 'more/scenarios/3/'
    map_files = [f for f in listdir(map_folder) if isfile(join(map_folder, f))]

    # agents_limits = [5, 10, 15, 20, 25]
    # conflict_bounds = [0, 1, 5, 10, 100]
    agents_limits = [5, 10]
    conflict_bounds = [1, 5]
    for map_file_name in map_files:
        if map_file_name == 'DS_Store':
            continue
        print("***Import an instance***")
        map_file = join(map_folder, map_file_name)
        scen_file = join(scen_folder, map_file_name + ".scen")
        # print(map_file, scen_file)
        # exit()
        for agents_limit in agents_limits:
            
            my_map, starts, goals = import_mapf_instance(map_file, scen_file, agents_limit=agents_limit)
            # print_mapf_instance(my_map, starts, goals)

            if args.solver == "all":
                print("***Run CBS***")
                for solver_idx, solver in enumerate([CBSSolver, MetaAgCBSWithCBS, MetaAgCBSWithMstar]):
                    if solver_idx == 0:
                        print("map file {}, scen file {}, solver idx {}, agent limit {}, conflict bound {},   \n".format(map_file, scen_file, solver_idx, agents_limit, '-'))
                        try:
                            sol = solver(my_map, starts, goals, args.merge_thresh)
                            (paths, time, n_exp, n_gen) = sol.find_solution()
                            cost = get_sum_of_cost(paths)
                            result_file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(solver.__name__, cost, time, n_exp, n_gen, map_file, scen_file, agents_limit, '-', 'success'))
                        except Exception as e:
                            print('Fail:', e)
                            if(str(e).split('|')[0] == 'timeout'):
                                result_file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(solver.__name__, '-', '-', str(e).split('|')[1], str(e).split('|')[2], map_file, scen_file, agents_limit, '-', 'timeout'))
                            else:
                                result_file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(solver.__name__, '-', '-', '-', '-', map_file, scen_file, agents_limit, '-', 'Fail:'+str(e)))
                    else:
                        for conflict_bound in conflict_bounds:
                            try:
                                print("map file {}, scen file {}, solver idx {}, agent limit {}, conflict bound {},   \n".format(map_file, scen_file, solver_idx, agents_limit, conflict_bound))
                                sol = solver(my_map, starts, goals, conflict_bound)
                                (paths, time, n_exp, n_gen) = sol.find_solution()
                                cost = get_sum_of_cost(paths)
                                result_file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(solver.__name__, cost, time, n_exp, n_gen, map_file, scen_file, agents_limit, conflict_bound, 'success'))
                            except Exception as e:
                                print('Fail:', e)
                                if(str(e).split('|')[0] == 'timeout'):
                                    result_file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(solver.__name__, '-', '-', str(e).split('|')[1], str(e).split('|')[2], map_file, scen_file, agents_limit, conflict_bound, 'timeout'))
                                else:
                                    result_file.write("{}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(solver.__name__, '-', '-', '-', '-', map_file, scen_file, agents_limit, conflict_bound, 'Fail:'+str(e)))
            if args.solver == "CBS":
                print("***Run CBS***")
                cbs = CBSSolver(my_map, starts, goals, args.merge_thresh)
                (paths, time, n_exp, n_gen) = cbs.find_solution()
            elif args.solver == "MetaCBSwCBS":
                print("***Run MetaCBS with CBS as low level solver***")
                ma_cbs = MetaAgCBSWithCBS(my_map, starts, goals, args.merge_thresh)
                (paths, time, n_exp, n_gen) = ma_cbs.find_solution()
                print("***Finished Run MetaCBS with CBS as low level solver***")
            elif args.solver == "MetaCBSwMstar":
                print("***Run MetaCBS with Mstar as low level solver***")
                ma_mstar = MetaAgCBSWithMstar(my_map, starts, goals, args.merge_thresh)
                (paths, time, n_exp, n_gen) = ma_mstar.find_solution()
                print("***Finished Run MetaCBS with Mstar as low level solver***")
            
            if args.solver != "all":
                cost = get_sum_of_cost(paths)
                result_file.write("{}, {}, {}, {}, {}, {}, {}\n".format(args.solver, cost, time, n_exp, n_gen, map_file.split('/')[2], scen_file.split('/')[2]))

        if args.animation:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            animation.show()
    result_file.close()