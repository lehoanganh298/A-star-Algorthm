import sys
import time
from grid_graph import Grid_graph

# Get grid size, grid matrix, start and goal point from input file
def input_from_file(file_name):
    try:
        with open(file_name) as f:
            size = int(f.readline())
            start = tuple([int(x) for x in f.readline().split()])
            goal = tuple([int(x) for x in f.readline().split()])
            assert size > 0 and len(start) == 2 and len(goal) == 2, 'size/start/goal is not correct'

            grid = []
            for row in range(size):
                line = [x for x in f.readline().split()]
                assert len(line) == size, f'row {row} of the grid is not correct'
                grid.append(line)

        return size, grid, start, goal

    except FileNotFoundError:
        raise Exception(f'File {file_name} does not exist.abc')
    except ValueError:
        raise Exception('Input file is not in the right format.')
    else:
        raise Exception('Some error occur.')

# BEGIN THE PROGRAM ---------------------------------------

# Get the grid space information and position of the Start cell and Goal cell
size, grid, start, goal = input_from_file('Test1.txt')

g = Grid_graph(grid)

### TEST UNIFORM COST SEARCH: print end result
# shortest_distance, path = g.uniform_cost_search(start, goal)
# g.print_search_result(shortest_distance, path)

### TEST UNIFORM COST SEARCH: print graph state over time
# shortest_distance, path = g.uniform_cost_search(start, goal, Grid_graph.print_grid_state)
# g.print_search_result(shortest_distance, path)

### TEST A* SEARCH: heuristic using Eulidean distance fomula
# shortest_distance, path = g.A_star_search(start, goal, 
#     distance_function=Grid_graph.euclidean_distance,
#     update_function= Grid_graph.print_grid_state)
# g.print_search_result(shortest_distance, path)

### TEST A* SEARCH: heuristic using my own distance function
# shortest_distance, path = g.A_star_search(start, goal, 
#     distance_function=Grid_graph.my_distance,
#     update_function= Grid_graph.print_grid_state)
# g.print_search_result(shortest_distance, path)

### TEST ARA* SEARCH: Search within given time limit (set to 10s)
# Note: For better illustration, add 0.3s each iteration of the search process
# def delay_each_iteration(self, frontier, start, goal):
#     time.sleep(0.3)
# shortest_distance, path = g.ARA_star_search(start, goal, 10,
#     update_session=Grid_graph.print_session_result,
#     update_iteration=delay_each_iteration)

### TEST PRINT TO FILE: print result A* search to file
# shortest_distance,path = g.A_star_search(start, goal)
# orig_stdout=sys.stdout
# sys.stdout=open('output.txt','w')
# g.print_search_result(shortest_distance, path)
# sys.stdout=orig_stdout