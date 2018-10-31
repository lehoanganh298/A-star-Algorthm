import heapq
import numpy as np

class Vertice:
    """
    Vertice object store information of each vertice in graph
    Store in adjacency list approach (each vertice have a list of adjacent vertices)
    """
    def __init__(self,adj_list=[]):
        self.adj_list=adj_list

class Graph:
    """
    A general graph compatible with many data structures and implementations
    Use adjacency list approach:
    Store a list of all vertices, each vertice have a list of adjacent vertices
    """
    def __init__(self):
        pass

    def vertice(self,vert):
        """ vert: vertice identifier (name/index)
        Return a Vertice object in this graph 
        """
        pass

    def vertice_list(self):
        """ A list or an iterator object use for iterate through the graph
            for vertice in vertice_list:
        """
        pass

    def uniform_cost_search(self, start, goal):
        """
        Find the shortest path from one vertice to another vertice in the graph
        using Uniform cost search (UCS) algorithm
        Main idea: expand the vertice with smallest distance in the frontier

        Input: 
        start, goal: identifiers (name/index) of the start point and the destination point

        Output:
        shortest_distance: the length of the shortest path from Start to Goal
        If there are no path from Start to Goal, shortest_path = -1
        path: list of vertices form the shortest path from Start to Goal
        """
        shortest_distance = -1  # shortest distance from start to goal
        path = []  # store vertices in the shortest road from start to goal

        inf = float('inf')    # just a name for the infinity number
        # Set distance of start vertice to 0, and other's to inf
        for ver in self.vertice_list():
            ver.dist = inf
        self.vertice(start).dist = 0

        frontier = []  # priority queue store CURRENTLY OPEN vertices and their distance
        # sorted by their distance (vertice with smallest distance on top)
        heapq.heappush(frontier, (0,start))  # add start vetice

        while frontier:
            dist , vert = heapq.heappop(frontier)

            if vert == goal:
                shortest_distance = self.vertice(goal).dist
                # Trace back to start and add vertices to {path}
                path = [goal]+path
                while vert != start:
                    vert = self.vertice(vert).trace
                    path = [vert]+path
                break

            for adj, edge_len in self.vertice(vert).adj_list:
                if self.vertice(adj).dist == inf \
                or self.vertice(adj).dist > self.vertice(vert).dist + edge_len:
                    self.vertice(adj).dist = self.vertice(vert).dist + edge_len   # update distance
                    self.vertice(adj).trace = vert # update trace back vertice
                    heapq.heappush(frontier, (self.vertice(adj).dist, adj))    # add to frontier

        return shortest_distance, path

class Grid(Graph):
    """
    A special type of graph. A grid of mxn cell, each cell is either valid (0) or unvalid (1)
    Find path go through valid cells from start point to destination point
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
        """ Transform the grid information to graph form (vertices and adjacent list) """
        # Initialize vertice_matrix same size with grid
        self.vertice_matrix=np.empty_like(grid,dtype=object)

        # Order when look for adjacent vertices: CLOCKWISE
        adj_order = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]
        for x, row in enumerate(grid):
            for y, point in enumerate(row):
                adj_list=[]
                if point=='0':
                    # Build the adj_list
                    for dx, dy in adj_order:
                        adj_x = x+dx
                        adj_y = y+dy
                        if adj_x>=0 and adj_x<grid.shape[0] and adj_y>=0 and adj_y<grid.shape[1] \
                        and grid[adj_x][adj_y]=='0':
                            adj_list.append(((adj_x,adj_y),1))  # distance between 2 adjacent vetices is 1

                self.vertice_matrix[x,y] = Vertice(adj_list)

    def vertice(self,vert):
        """ Vertice identifier is a tuple (x,y) correspoding to the vertice's position """
        return self.vertice_matrix[vert]

    def vertice_list(self):
        # Just return the list of all element in the matrix
        return [v for row in self.vertice_matrix for v in row]
