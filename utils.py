from os.path import isfile, join
import os

results_folder = 'results'
result_file = 'results3.csv'
path = join(results_folder, result_file)
outpath = join(results_folder,result_file.rsplit('.', 1)[0] + "_table.csv")
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
    out.write(',' + l[2].strip())
out.close()