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

        frontier = {}  # priority queue store CURRENTLY OPEN vertices and their distance
        # sorted by their distance (vertex with smallest distance on top)
        frontier[start] = priority_function(start)  # add start vetice

        while frontier:
            # print current graph state or any update operation in each iteration
            update_function(self, frontier, start, goal)

            # pop vertex with smallest priority value value
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
                    self.vertex(adj).dist = self.vertex(
                        vert).dist + edge_len   # update distance
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

    def ARA_star_search(self, start, goal, heuristic_function, 
        update_session=lambda e,d,p: None,
        update_iteration=lambda gh, f, s, g: None):

        def priority_function(vert,epsilon):
            return self.vertex(vert).dist + epsilon*heuristic_function(vert)

        def improve_path(epsilon):
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
                    # Trace back to start and add vertices to {path}
                    path = [goal] + path
                    while vert != start:
                        vert = self.vertex(vert).trace
                        path = [vert]+path
                    break

                for adj, edge_len in self.vertex(vert).adjacency_list:
                    if self.vertex(adj).dist == -1 \
                            or self.vertex(adj).dist > self.vertex(vert).dist + edge_len:
                        self.vertex(adj).dist = self.vertex(
                            vert).dist + edge_len   # update distance
                        self.vertex(adj).trace = vert  # update trace back vertex
                        # update/add updated vertex to frontier with it's priority key value
                        frontier[adj] = priority_function(adj,epsilon)

            return self.vertex(goal).dist, path, frontier

        path = []  # store vertices in the shortest road from start to goal
        self.vertex(start).dist = 0
        epsilon=7
        frontier = {}  # priority queue store CURRENTLY OPEN vertices and their distance
        # sorted by their distance (vertex with smallest distance on top)
        frontier[start] = priority_function(start,epsilon)  # add start vetice

        shortest_distance, path, frontier = improve_path(epsilon)
        update_session(epsilon,shortest_distance,path)

        while epsilon>1:
            epsilon-=1
            for vert in frontier:
                frontier[vert]=priority_function(vert,epsilon)
            frontier[goal]=priority_function(goal,epsilon)
            shortest_distance, path, frontier = improve_path(epsilon)
            update_session(epsilon,shortest_distance,path)
        
        return shortest_distance, path

    # def improve_path(self, goal, frontier, 
    #         priority_function, update_function=lambda gh, f, s, g: None):
    #     def pop_minimum(dic):
    #         min_key = 0
    #         for idx, key in enumerate(dic):
    #             if idx == 0 or priority_function(key)<priority_function(min_key):
    #                 min_key = key
    #         dic.remove(min_key)
    #         return min_key

    #     closed_queue=incons_queue = set()
    #     while priority_function(goal)>min([priority_function(v) for v in frontier]):
    #         update_function(frontier,start,goal)
    #         vert = pop_minimum(frontier)
    #         closed_queue.add(vert)
    #         for adj, edge_len in self.vertex(vert).adjacency_list:
    #             if self.vertex(adj).dist == -1 \
    #                 or self.vertex(adj).dist > self.vertex(vert).dist + edge_len:
    #                 self.vertex(adj).dist = self.vertex(vert).dist + edge_len   # update distance
    #                 self.vertex(adj).trace = vert  # update trace back vertex
    #                 # update/add updated vertex to frontier with it's priority key value
    #                 if adj not in closed_queue:
    #                     frontier.add(adj)
    #                 else:
    #                     incons_queue.add(adj)

    #     return self.vertex(goal).dist, frontier, incons_queue

    # def ARA_star_search(self, time_limit, start, goal ,heuristic_function, 
    #     update_session=lambda d,p: None,
    #     update_function=lambda gh, f, s, g: None):
    #     """
    #     ARA* (Anytime repairing A*) search algorithm (search within a given time limit)
    #     Search with heuristic h'(v) = eps*h(v) (eps>1) to find a path quicker but less optimal.
    #     If still have time, decrease eps and continue another search session to find a more optimal solution.
    #     """
    #     def fvalue(vert):
    #         return self.vertex(vert).dist + epsilon*heuristic_function(vert)

    #     shortest_distance=-1
    #     path=[]
    #     epsilon_initial_value = 10  # initial value of epsilon
    #     epsilon_delta = 1           # epsilon decrease amount each session

    #     time_begin = time.time()
    #     epsilon = epsilon_initial_value
    #     open_queue = incons_queue=set()
    #     open_queue.add(start)
        
    #     shortest_distance, open_queue, incons_queue \
    #     = self.improve_path(goal, open_queue, fvalue, update_function)

    #     if shortest_distance == -1:
    #         return shortest_distance

    #     sub_opt = min(epsilon, shortest_distance/ min([self.vertex(v).dist+heuristic_function(v) for v in {**open_queue,**incons_queue}]))
    #     update_session(sub_opt)

    #     while sub_opt>1 and time.time()-time_begin<time_limit:
    #         epsilon-=epsilon_delta
    #         open_queue={**open_queue,**incons_queue}
    #         self.improve_path(goal, open_queue,epsilon,update_function)

    #         sub_opt = min(epsilon, shortest_distance/ \
    #             min([self.vertex(v).dist+heuristic_function(v) \
    #                 for v in {**open_queue,**incons_queue}]))
    #         update_session(sub_opt)

    #     return shortest_distance, path
            
        