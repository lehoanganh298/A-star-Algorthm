import sys

class Point:
    """ store information of each point in the grid space """
    def __init__(self,valid,dist,trace=None):
        self.valid=valid;   # true=empty, false=obstacle
        self.dist=dist;     # distance from the start point
        self.trace=trace;   # store previous point in the path to the goal

class Graph:
    """
    Graph class store information of the grid space:
        matrix: store points in the grid
        size: size of the grid (size x size)
        start: position of the start point (x,y)
        goal: position of the goal point (x,y)
    """
    def __init__(self):
        pass
    
    # Read input 
    # Remember to replace input.txt to sys.argv[1]
    def input(self):
        with open('input.txt') as f:
            self.size, = [int(x) for x in next(f).split()]
            self.start = tuple([int(x) for x in next(f).split()])
            self.goal = tuple([int(x) for x in next(f).split()])
            
            self.matrix=[]
            maxPathLength=self.size*2; # path from start to goal can not exceed size*2
            # Read the matrix
            # 0 -> Point(True,maxPathLength,None)
            # 1 -> Point(False,-1,None)
            for line in f:
                self.matrix.append([Point(True,maxPathLength) \
                    if x=='0' else Point(False,-1) \
                  for x in line.split()])
        
        # Distance of start point is 0
        self.point(self.start).dist=0;
        # Mark that start point's trace is not None, but is not other position
        self.point(self.start).trace=(-1,-1);
    
    
    # return the matrix element of graph at position pos
    def point(self,pos):
        return self.matrix[pos[0]][pos[1]];
    
    
    # Output
    # Remember to replace output.txt to sys.argv[2]  
    #sys.stdout=open('output.txt','w')
    def output(self):
        for row in self.matrix:
            for i in row:
                print(i.dist, end=' ');
            print();
     
    # true if pos is an empty point
    def validPos(self,pos):
        return pos[0]>=0 and pos[0]<self.size and \
            pos[1]>=0 and pos[1]<self.size and \
            self.point(pos).valid;
                
    def BFS(self):
        adjOrder=[(-1,-1),(-1,0),(-1,1),(0,1),(1,1),(1,0),(1,-1),(0,-1)];
        
        queue=[];
        
        queue.append(self.start);
        
        while queue:
            p=queue.pop(0);
            if p==self.goal:
                while p!=self.start:
                    print(f'({p[0]},{p[1]}) <-',end='');
                    p=self.point(p).trace;
                print(f'({p[0]},{p[1]})')
            
            for adj in adjOrder:
                padj=(p[0]+adj[0],p[1]+adj[1]);
                if self.validPos(padj) and self.point(padj).trace==None:
                    self.point(padj).dist=self.point(p).dist+1;
                    self.point(padj).trace=p;
                    queue.append(padj);
            
g=Graph();
g.input();
g.BFS();
#g.output();