import time

class Vertex:
    """
    Vertex object store information of each vertex in graph
    Store in adjacency list approach (each vertex have a list of adjacent vertices)
    """

    def __init__(self, adj_list=[]):
        self.adjacency_list = adj_list
        self.dist = -1        # distance value of each vertex used in search algorithm, default -1
        self.trace = None     # vertex to trace back to start point in finding path in graph


class Graph:
    """
    A general graph compatible with many data structures and implementations
    Use adjacency list approach:
    Store a list of all vertices, each vertex have a list of adjacent vertices
    """

    def __init__(self):
        pass

    def vertex(self, vert):
        """ vert: vertex identifier (name/index)
        Return a vertex object in this graph 
        """
        pass

    def trace_back_path(self,start, goal):
        """ From graph state after seaching process, 
        trace back from goal vertex to start using the trace attribute to find the path """
        path = [goal]
        vert=goal
        while vert != start:
            vert = self.vertex(vert).trace
            path = [vert]+path
        return path

    def search_with_priority_queue(self, start, goal, priority_function, update_function=lambda gh, f, s, g: None):
        """
        Find the shortest path from one vertex to another vertex in the graph using priority queue
        Main idea: expand the vertex with smallest priority_function value in the frontier

        Input: 
        start, goal: identifiers (name/index) of the start point and the destination point
        priority_function: return a priority value to each vertex to determine which vertex in the frontier to be expanded
            (expand vertex with smallest priority value)
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
        #####################################################################
        # TODO: implement frontier using heap (currently using dictionary)  #
        #####################################################################
        # Return the key that have the minimum value in the dictionary
        def pop_minimum(dic):
            min_key = 0
            for idx, key in enumerate(dic):
                if idx == 0 or dic[key] < dic[min_key]:
                    min_key = key
            dic.pop(min_key)
            return min_key

        path = []  # store vertices in the shortest road from start to goal
        self.vertex(start).dist = 0

        frontier = {}  # priority queue store CURRENTLY OPEN vertices and their priority value
        # sorted by their priority value (vertex with smallest priority value on top)
        frontier[start] = priority_function(start)  # add start vertex

        while frontier:
            # print current graph state or any update operation in each iteration
            update_function(self, frontier, start, goal)

            # pop vertex with smallest priority value value
            vert = pop_minimum(frontier)

            if vert == goal:
                path = self.trace_back_path(start, goal)
                break

            for adj, edge_len in self.vertex(vert).adjacency_list:
                if self.vertex(adj).dist == -1 \
                        or self.vertex(adj).dist > self.vertex(vert).dist + edge_len:
                    self.vertex(adj).dist = self.vertex(vert).dist + edge_len   # update distance
                    self.vertex(adj).trace = vert  # update trace back vertex
                    # update/add updated vertex to frontier with it's priority key value
                    frontier[adj] = priority_function(adj)

        return self.vertex(goal).dist, path

    def uniform_cost_search(self, start, goal, update_function=lambda gh, f, s, g: None):
        """ 
        Uniform cost search (UCS) / Dijkstra algorithm
        Expand the vertex with smallest dist value in the frontier
        """
        return self.search_with_priority_queue(start, goal,
                                                priority_function=lambda vert: self.vertex(vert).dist,
                                                update_function=update_function)

    def greedy_search(self, start, goal, heuristic_function, update_function=lambda gh, f, s, g: None):
        """
        Greedy seach algorithm
        Use the heuristic_function to estimate distance of a vertex to the Goal
        Expand the vertex in the frontier with smallest heuristic value
        Very fast performance
        Result path may not be optimal (shortest)
        """
        return self.search_with_priority_queue(start, goal,
                                               priority_function=heuristic_function,
                                               update_function=update_function)
                                               
    def A_star_search(self, start, goal, heuristic_function, update_function=lambda gh, f, s, g: None):
        """
        A* algorithm: search with extra information
        Use the heuristic_function to estimate distance of a vertex to the Goal
        Expand the vertex in the frontier with the smallest f(vert) = dist(vert) + heuristic(vert)
        """
        def f_A_star(vert):
            return self.vertex(vert).dist + heuristic_function(vert)

        return self.search_with_priority_queue(start, goal,
                                               priority_function=f_A_star,
                                               update_function=update_function)

    def ARA_star_search(self, start, goal, time_limit, heuristic_function, 
        epsilon = 10,
        update_session=lambda e,d,p: None,
        update_iteration=lambda gh, f, s, g: None):
        """
        ARA* (Anytime repairing A*) search algorithm (search within a given time limit)
        Search with heuristic h'(v) = eps*h(v) (eps>1) to find a path quick but not optimal.
        If still have time, decrease eps and continue another search session to find a more optimal solution.
        """

        def priority_function(vert):
            return self.vertex(vert).dist + epsilon*heuristic_function(vert)

        def improve_path():
            def pop_minimum(dic):
                min_key = 0
                for idx, key in enumerate(dic):
                    if idx == 0 or dic[key] < dic[min_key]:
                        min_key = key
                dic.pop(min_key)
                return min_key
            
            path=[]
            while frontier:
                # print current graph state or any update operation in each iteration
                update_iteration(self, frontier, start, goal)
                # pop vertex with smallest priority value value
                vert = pop_minimum(frontier)
                if vert == goal:
                    path = self.trace_back_path(start, goal)
                    break

                for adj, edge_len in self.vertex(vert).adjacency_list:
                    if self.vertex(adj).dist == -1 \
                            or self.vertex(adj).dist > self.vertex(vert).dist + edge_len:
                        self.vertex(adj).dist = self.vertex(vert).dist + edge_len   # update distance
                        self.vertex(adj).trace = vert  # update trace back vertex
                        # update/add updated vertex to frontier with it's priority key value
                        frontier[adj] = priority_function(adj)

            return self.vertex(goal).dist, path, frontier

        time_begin=time.time() # get the time at function begin execute

        path = []  # store vertices in the shortest road from start to goal
        self.vertex(start).dist = 0
        frontier = {}  # priority queue store CURRENTLY OPEN vertices and their priority value
        frontier[start] = priority_function(start)  # add start vertex

        shortest_distance, path, frontier = improve_path()
        if time.time()-time_begin<time_limit:
            update_session(self,time.time()-time_begin,epsilon,shortest_distance,path) # print result of the fitrst session

            if shortest_distance != -1:
                while epsilon>1 and time.time()-time_begin<time_limit:
                    epsilon-=1  # decrease epsilon

                    # update frontier elements with new priority_function (new epsilon)
                    for vert in frontier:
                        frontier[vert]=priority_function(vert)
                    frontier[goal]=priority_function(goal)  # add goal to the frontier

                    # improve path in another session with new epsilon value
                    shortest_distance, path, frontier = improve_path()
                    if time.time()-time_begin<time_limit:
                        update_session(self,time.time()-time_begin,epsilon,shortest_distance,path)
                    else:
                        break
        
        return shortest_distance, path
