from os.path import isfile, join
import os

results_folder = 'results'
for k in range(4,7):
    for table_type in ['time', 'exp', 'gen']:
        result_file = 'results' + str(k) + 'ost.csv'
        path = join(results_folder, result_file)
        outpath = join(results_folder,result_file.rsplit('.', 1)[0] + '_table_' + table_type + '.csv')
        out = open(outpath, "w+", buffering=1)

        agents_limits = [5, 10, 15, 20]
        conflict_bounds = [1, 5, 10, 100]
        f= open(path,'r')
        line = f.readlines()
        for i in range(2, len(line)):
            l = line[i].split(',')
            if l[-3].strip() == '25':
                continue
            if (i-2) % 25 == 0:
                if i != 2:
                    out.write('\n\n')
                out.write(l[5].split('/')[-1])
                out.write('\nk,CBS,B(1),B(5),B(10),B(100)')
            if l[0] == 'CBSSolver':
                out.write('\n')
                out.write(str((i-2) % 25 + 5))
            out.write(',')
            if l[-1].strip() == 'timeout':
                out.write('>')
            if table_type == 'time':
                out.write(l[2].strip())
            elif table_type == 'exp':
                out.write(l[3].strip())
            else:
                out.write(l[4].strip())
        out.close()