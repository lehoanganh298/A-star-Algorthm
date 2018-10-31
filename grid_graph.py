import numpy as np
from graph import Vertice, Graph

class Cell(Vertice):
    """ A cell in a Grid graph, is either a valid (emty cell) or unvalid (obstacle) """
    def __init__(self, valid,adj_list=[]):
        Vertice.__init__(self,adj_list)
        self.valid=valid 

class Grid_graph(Graph):
    """
    A special type of graph. A grid of mxn cells, each cell is either valid (0) or unvalid (1)
    Find path go through valid cells from start cell to destination cell
    From each cell can go to adjacent cell have either commond edge or common vertice
    Each cell can be treated as a vertice, and connected with  adjacent vetices

    Data structure:
    Use a matrix {vertice_matrix} to store every vertices in the grid
    Each element is a Vertice object
    Add valid adjacent vertices to each valid vertice's adj_list

    Implementation:
    Use numpy library for better data structures and operations
    vertice_matrix is a numpy array
    """
    def __init__(self,grid):
        """ 
        From a grid matrix of 0 (valid) and 1(unvalid) cells, 
        initialize the vertice_matrix and each vertice's adjacent list
        """
        self.size=(len(grid),len(grid[0]))  # size of the grid
        # Initialize vertice_matrix same size with grid
        self.vertice_matrix=np.empty(self.size,dtype=object)

        # Order when look for adjacent vertices: CLOCKWISE
        adj_order = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
        for x, row in enumerate(grid):
            for y, cell in enumerate(row):
                adj_list=[]
                if cell=='0':
                    # Build the adj_list
                    for dx, dy in adj_order:
                        adj_x = x+dx
                        adj_y = y+dy
                        if adj_x>=0 and adj_x<self.size[0] and adj_y>=0 and adj_y<self.size[1] \
                        and grid[adj_x][adj_y]=='0':
                            adj_list.append(((adj_x,adj_y),1))  # distance between 2 adjacent vetices is 1
                
                self. vertice_matrix[x,y] = Cell(valid = (cell=='0'), adj_list = adj_list)

    def vertice(self,vert):
        """ Vertice identifier is a tuple (x,y) correspoding to the vertice's position """
        return self.vertice_matrix[vert]

    def print_found_path(self, shortest_distance, path):
        print(shortest_distance)

        if shortest_distance > 0:
            grid_output=np.full(self.size,'-')
            for x in range(self.size[0]):
                for y in range(self.size[1]):
                    if self.vertice((x,y)).valid==False:
                        grid_output[x,y]='o'

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

    # Print current grid_graph state with frontier and cells's distances
    def print_grid_state(self,frontier,start,goal):
        grid_state=np.full(self.size,' --')

        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if self.vertice((i,j)).valid==False:
                    grid_state[i,j]='[]'
                elif self.vertice((i,j)).dist != -1:
                    grid_state[i,j]=self.vertice((i,j)).dist
                    
        grid_state[start]='S'
        grid_state[goal]='G'
        for key, vert in frontier:
            grid_state[vert]=f'*{self.vertice(vert).dist}'
        
        print('------------------------------')
        for row in grid_state:
            for cell in row:
                print('%3s' % (cell),end='')
            print()
        input()

