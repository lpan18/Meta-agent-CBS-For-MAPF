import copy
import bisect

def convert_cons(node):
    groups = node['groups']
    grouptl = {}
    for constraint in node['constraints']:
        # group = tuple(groups[constraint['agent']]) # need to change
        group = tuple(groups[constraint['group']])
        time = constraint['timestep']
        loc = tuple(constraint['loc'],)
        tl = (time,loc)
        if group in grouptl:
            grouptl[group].append(tl)
        else:
            grouptl[group] = [tl]

    cons = []
    for k in grouptl:
        c = (k,tuple(grouptl[k]))
        cons.append(c)
    cons = tuple(sorted(cons))
    return cons

def con_get_robots(constraint):
    con = copy.deepcopy(constraint)
    if len(constraint) == 1:
        con = con[0]
    return con[0]

def con_subset_robots(constraint, robots):
    con = copy.deepcopy(constraint)
    if len(constraint) == 1:
        con = con[0]
    return (tuple(sorted(robots)), con[1])

def con_get_max_time(constraint):
    con = copy.deepcopy(constraint)
    if len(constraint) == 1:
        con = con[0]
    if len(con[1]) == 0:
        return 0
    return con[1][-1][0]

def con_is_constrained(constraint, time, coord1, coord2):
    dex = bisect.bisect_left(constraint[1], (time, ))
    while (dex < len(constraint[1]) and
           constraint[1][dex][0] == time):
        sub = constraint[1][dex]
        if sub[0] == time:
            if len(sub[1]) == 2:
                if (sub[1][0] == coord1 and
                    sub[1][1] == coord2):
                    return True
            else:
                if (sub[1][0] == coord2):
                    return True
        dex += 1
    return False

def convert_path(path):
    new_path = [ [] for _ in range(len(path[0]))]
    for p in path:
        for i in range(len(p)):
            new_path[i].append(p[i])
    return new_path

if __name__ == '__main__':
    path = (((2, 2), (2, 3)), ((2, 1), (2, 4)), ((1, 1), (2, 3)), ((2, 1), (2, 2)), ((2, 2), (1, 2)), ((2, 3), (1, 2)), ((2, 4), (1, 2)), ((1, 4), (1, 2)), ((1, 5), (1, 2)))
    path = convert_path(path)
