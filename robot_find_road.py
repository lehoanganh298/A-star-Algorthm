import sys
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
#shortest_distance, path = g.uniform_cost_search(start, goal, Grid_graph.print_grid_state)
# shortest_distance, path = g.greedy_search(start, goal, 
#     distance_function=Grid_graph.my_distance,
#     update_function= Grid_graph.print_grid_state)
shortest_distance, path = g.A_star_search(start, goal, 
    distance_function = Grid_graph.my_distance,
    update_function = Grid_graph.print_grid_state)

# orig_stdout=sys.stdout
# sys.stdout=open('output.txt','w')
g.print_search_result(shortest_distance, path)
# sys.stdout=orig_stdout
