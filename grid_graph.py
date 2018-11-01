import numpy as np
import math
import time
from graph import Vertex, Graph


class Cell(Vertex):
    """ A cell in a Grid graph, is either a valid (emty cell) or unvalid (obstacle) """

    def __init__(self, valid, adj_list=[]):
        Vertex.__init__(self, adj_list)
        self.valid = valid


class Grid_graph(Graph):
    """
    A special type of graph. A grid of mxn cells, each cell is either valid (0) or unvalid (1)
    Find path go through valid cells from start cell to destination cell
    From each cell can go to adjacent cell have either commond edge or common vertex
    Each cell can be treated as a vertex, and connected with  adjacent vetices

    Data structure:
    Use a matrix {vertex_matrix} to store every vertices in the grid
    Each element is a Vertex object
    Add valid adjacent vertices to each valid vertex's adj_list

    Implementation:
    Use numpy library for better data structures and operations
    vertex_matrix is a numpy array
    """

    def __init__(self, grid):
        """ 
        From a grid matrix of 0 (valid) and 1(unvalid) cells, 
        initialize the vertex_matrix and each vertex's adjacent list
        """
        self.size = (len(grid), len(grid[0]))  # size of the grid
        # Initialize vertex_matrix same size with grid
        self.vertex_matrix = np.empty(self.size, dtype=object)

        # Order when look for adjacent vertices: CLOCKWISE
        adj_order = [(-1, -1), (-1, 0), (-1, 1), (0, 1),
                     (1, 1), (1, 0), (1, -1), (0, -1)]
        for x, row in enumerate(grid):
            for y, cell in enumerate(row):
                adj_list = []
                if cell == '0':
                    # Build the adj_list
                    for dx, dy in adj_order:
                        adj_x = x+dx
                        adj_y = y+dy
                        if adj_x >= 0 and adj_x < self.size[0] and adj_y >= 0 and adj_y < self.size[1] \
                                and grid[adj_x][adj_y] == '0':
                            # distance between 2 adjacent vetices is 1
                            adj_list.append(((adj_x, adj_y), 1))

                self. vertex_matrix[x, y] = Cell(valid=(cell == '0'), adj_list=adj_list)

        # variable to count number of graph update iterations
        # used in update_function
        self.iteration_count = 0 

    def vertex(self, vert):
        """ Vertex identifier is a tuple (x,y) correspoding to the vertex's position """
        return self.vertex_matrix[vert]

    @staticmethod
    def euclidean_distance(a,b):
        """ 
        A way to measure of distance between vertex a and vertex b in the grid
        Use Euclidean distance fomula: d = sqrt((a.x-b.x)^2 + (a.y-b.y)^2)
        """
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

    @staticmethod
    def my_distance(a, b):
        """ 
        A way to measure of distance between vertex a and vertex b in the grid
        Use distance fomula: d = max(|a.x-b.x|, |a.y-b.y|)
        """
        return max(abs(a[0]-b[0]), abs(a[1]-b[1]))

    def greedy_search(self, start, goal, distance_function = my_distance.__func__, update_function=lambda gh, f, s, g: None):
        """
        Overload of Graph.greedy_search():
        Input a distance_function (function to measure distance bettween 2 cell)
        Calculate heuristic function and feed to Graph.greedy_search()
        """
        return Graph.greedy_search(self, start, goal,
                                   heuristic_function=lambda vert: distance_function(vert, goal),
                                   update_function=update_function)

    def A_star_search(self, start, goal, distance_function = my_distance.__func__, update_function=lambda gh, f, s, g: None):
        """
        Overload of Graph.A_star_search():
        Input a distance_function (function to measure distance bettween 2 cell)
        Calculate heuristic function and feed to Graph.A_star_search()
        """
        return Graph.A_star_search(self, start, goal,
                                   heuristic_function=lambda vert: distance_function(vert, goal),
                                   update_function=update_function)

    def print_grid_state(self, frontier, start, goal):
        """ 
        Print current grid_graph state:
        frontier: cells with *, display priority_key value
        cell about to expand: cel with +, display priority_key value
        expanded cells: display distance value
        obstacles: display []
        Stop between each execution: input() or time.sleep() at the end of function
         """
        self.iteration_count += 1
        grid_state = np.full(self.size, ' --')

        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if self.vertex((i, j)).valid == False:
                    grid_state[i, j] = '[]'
                elif self.vertex((i, j)).dist != -1:
                    grid_state[i, j] = self.vertex((i, j)).dist

        grid_state[start] = 'S'
        grid_state[goal] = 'G'

        min_vert = 0 # min_vert is vertex with smallest priority value, 0 is just the initialize value
        for idx, vert in enumerate(frontier):
            #print(f'{vert}:{frontier[vert]}',end=' ')
            if idx == 0 or frontier[vert] < frontier[min_vert]:
                min_vert = vert
            grid_state[vert] = f'*{frontier[vert]}'
        grid_state[min_vert] = f'+{frontier[min_vert]}'

        print()
        print('%3d|--------------------------|' % (self.iteration_count))
        for row in grid_state:
            for cell in row:
                print('%3s' % (cell), end='')
            print()
        input()
        #time.sleep(0.5)

    def print_found_path(self, path):
        """ Print the found shortest path in the grid: cells in path display by 'x' """
        grid_output = np.full(self.size, '-')
        for x in range(self.size[0]):
            for y in range(self.size[1]):
                if self.vertex((x, y)).valid == False:
                    grid_output[x, y] = 'o'

        for p in path:
            grid_output[p] = 'x'
        grid_output[path[0]] = 'S'
        grid_output[path[-1]] = 'G'

        for row in grid_output:
            for p in row:
                print(p, end=' ')
            print()
