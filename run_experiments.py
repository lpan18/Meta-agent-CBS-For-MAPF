#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from meta_agent_cbs_w_cbs import MetaAgCBSWithCBS
from meta_agent_cbs_w_mstar import MetaAgCBSWithMstar
from visualize import Animation
from single_agent_planner import get_sum_of_cost

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
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


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(1)
            elif cell == '.':
                my_map[-1].append(0)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    # print(my_map)
    return my_map, starts, goals


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('-i', dest='instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('-s', dest='solver', type=str, default="all",
                        help='The solver to use (CBS, MetaCBSwCBS, MetaCBSwMstar), defaults to run all solvers')
    parser.add_argument('-b', dest='merge_thresh', type=int, default=10,
                        help='Merge threshold for Meta-CBS')

    args = parser.parse_args()

    result_file = open("results.csv", "a", buffering=1)
    result_file.write("===================\n")
    result_file.write("solver, cost, time, n_exp, n_gen, instance_file \n")

    for file in sorted(glob.glob(args.instance)):
    # for i in range(48,51):
    #     file = "instances/test_" + str(i) + ".txt"
        try:
            print("***Import an instance***")
            print("instance file:", file)
            my_map, starts, goals = import_mapf_instance(file)
            # print_mapf_instance(my_map, starts, goals)

            if args.solver == "all":
                print("***Run CBS***")
                for solver in [CBSSolver,MetaAgCBSWithCBS, MetaAgCBSWithMstar]:
                    sol = solver(my_map, starts, goals, args.merge_thresh)
                    (paths, time, n_exp, n_gen) = sol.find_solution()
                    cost = get_sum_of_cost(paths)
                    result_file.write("{}, {}, {}, {}, {}, {} \n".format(solver.__name__, cost, time, n_exp, n_gen, file))
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
                result_file.write("{}, {}, {}, {}, {}, {} \n".format(solver.__name__, cost, time, n_exp, n_gen, file))
        
        except Exception as e:
            result_file.write("Exception {} on file {}\n".format(e, file))

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0)
            animation.show()
    result_file.close()
