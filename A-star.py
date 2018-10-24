import sys

# Read input
# Remember to replace input.txt to sys.argv[1]
with open('input.txt') as f:
    n, = [int(x) for x in next(f).split()] # read first line
    sx,sy = [int(x) for x in next(f).split()] # read first line
    gx,gy = [int(x) for x in next(f).split()] # read first line
    graph = []
    for line in f: # read rest of lines
        graph.append([int(x) for x in line.split()])

for i in graph:
    print(i);
#graph=[[[] for x in range(n)] for y in range(n)]
# Output
# Remember to replace output.txt to sys.argv[2]  
#sys.stdout=open('output.txt','w')
