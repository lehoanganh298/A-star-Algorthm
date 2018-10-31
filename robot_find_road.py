import sys
import numpy as np
from graph import Graph, Grid, Vertice

# Get  grid size, grid matrix, start and goal point from input file
def input(file_name):
    try:
        with open(file_name) as f:
            size = int(f.readline())
            start = tuple([int(x) for x in f.readline().split()])
            goal = tuple([int(x) for x in f.readline().split()])
            assert size>0 and len(start)==2 and len(goal)==2, 'size/start/goal is not correct'

            grid=np.full((size,size),'0')
            for row in range(size):
                line = [x for x in f.readline().split()]
                assert len(line)==size, f'row {row} is not correct'
                grid[row,:] = line
            
        return start, goal, grid

    except FileNotFoundError:
        raise Exception(f'File {file_name} does not exist.abc')
    except ValueError:
        raise Exception('Input file is not in the right format.')
    else:
        raise Exception('Some error occur.')

# Output the found shortest path and it's lenth
def output(file_out, shortest_distance, path, grid):
    orig_stdout=sys.stdout
    sys.stdout=file_out

    print(shortest_distance)

    if shortest_distance > 0:
        grid_output=np.empty_like(grid)
        for x, row in enumerate(grid):
            for y, point in enumerate(row):
                grid_output[x,y] = '-' if point=='0' else 'o'

        for p in path:
            grid_output[p] = 'x'
        grid_output[path[0]]='S'
        grid_output[path[-1]]='G'

        for p in path:
            print(f'({p[0]},{p[1]})', end=' ')
        print()
        for row in grid_output:
            for p in row:
                print(p, end=' ')
            print()

    sys.stdout=orig_stdout

g=Graph()
start, goal, grid = input('Test4.txt')
g = Grid(grid)
shortest_distance, path = g.uniform_cost_search(start, goal)
output(sys.stdout,shortest_distance,path,grid)
