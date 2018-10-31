class Vertex:
    """
    Vertex object store information of each vertex in graph
    Store in adjacency list approach (each vertex have a list of adjacent vertices)
    """
    def __init__(self,adj_list=[]):
        self.adjacency_list=adj_list 
        self.dist=-1        # distance value of each vertex used in search algorithm, default -1
        self.trace=None     # vertex to trace back to start point in finding path in graph

class Graph:
    """
    A general graph compatible with many data structures and implementations
    Use adjacency list approach:
    Store a list of all vertices, each vertex have a list of adjacent vertices
    """
    def __init__(self):
        pass

    def vertex(self,vert):
        """ vert: vertex identifier (name/index)
        Return a vertex object in this graph 
        """
        pass

    def search_with_priority_queue(self, start, goal, priority_function, update_function=lambda g,f,start,goal:None):
        """
        Find the shortest path from one vertex to another vertex in the graph using priority queue
        Main idea: expand the vertex with smallest distance in the frontier

        Input: 
        start, goal: identifiers (name/index) of the start point and the destination point
        priority_function: return a key value to each vertex to determine which vertex in the frontier to be expanded
            (expand vertex with smallest key value)
        update_function (optional): function execute each time the graph update
            (i.e expand a vertex in the frontier)
            Used for keep track and print graph changes overtime
            Parameter:
                self: this graph
                frontier: give update_function the frontier's current value
                start, goal: the start vertex and goal vertex

        Output:
        shortest_distance: the length of the shortest path from Start to Goal
            If there are no path from Start to Goal, shortest_path = -1
        path: list of vertices form the shortest path from Start to Goal
        """
        path = []  # store vertices in the shortest road from start to goal
        self.vertex(start).dist = 0

        #####################################################################
        # TODO: implement frontier using heap (currently using dictionary)  #
        #####################################################################
        def pop_minimum(dic):
            for idx, key in enumerate(dic):
                if idx==0 or dic[key]<dic[min_key]:
                    min_key=key
            dic.pop(min_key)
            return min_key

        frontier = {}  # priority queue store CURRENTLY OPEN vertices and their distance
        # sorted by their distance (vertex with smallest distance on top)
        frontier[start] = priority_function(start)  # add start vetice

        while frontier:
            update_function(self,frontier, start, goal)   #print current graph state or any update operation in each iteration
            
            vert = pop_minimum(frontier)

            if vert == goal:
                # Trace back to start and add vertices to {path}
                path = [goal]+path
                while vert != start:
                    vert = self.vertex(vert).trace
                    path = [vert]+path
                break

            for adj, edge_len in self.vertex(vert).adjacency_list:
                if self.vertex(adj).dist == -1 \
                or self.vertex(adj).dist > self.vertex(vert).dist + edge_len:
                    self.vertex(adj).dist = self.vertex(vert).dist + edge_len   # update distance
                    self.vertex(adj).trace = vert # update trace back vertex
                    # update/add updated vertex to frontier with it's priority key value
                    frontier[adj]=priority_function(adj)

        return self.vertex(goal).dist, path


    def uniform_cost_search(self, start, goal, update_function=lambda g,f,start,goal:None):
        return self.search_with_priority_queue(start, goal, 
        priority_function = lambda vert: self.vertex(vert).dist, 
        update_function=update_function)


    def A_star_search(self,start, goal, heuristic_function, update_function=lambda g,f,start,goal:None):
        def f_A_star(vert):
            return self.vertex(vert).dist + heuristic_function(vert)
        
        return self.search_with_priority_queue(start, goal, f_A_star, update_function)
