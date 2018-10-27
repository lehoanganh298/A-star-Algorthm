import sys
import heapq as heap
import math
from tkinter.ttk import Frame, Label
import tkinter.font as tkFont

class Point:
    """ store information of each point in the grid space """

    def __init__(self, valid, dist, trace=None):
        self.valid = valid  # true=empty, false=obstacle
        self.dist = dist  # distance from the start point
        self.trace = trace  # store previous point in the path to the goal


class Graph:
    """
    Graph class store information of the grid space:
        matrix: store points in the grid
    #         size: size of the grid (size x size)
    #         start: position of the start point (x,y)
    #         goal: position of the goal point (x,y)
    #     """

    def __init__(self):
        self.size = 0
        self.start = tuple()
        self.goal = tuple()
        self.matrix = []
        self.maxPathLength = 10000

    # Read input
    # Remember to replace input.txt to sys.argv[1]
    def input(self):
        with open('input.txt') as f:
            self.size = int(next(f))
            self.start = tuple([int(x) for x in next(f).split()])
            self.goal = tuple([int(x) for x in next(f).split()])

            # self.matrix = []
            maxPathLength = self.size * self.size  # path from start to goal can not exceed size*2
            # Read the matrix
            # 0 -> Point(True,maxPathLength,None)
            # 1 -> Point(False,-1,None)
            for line in f:
                self.matrix.append([
                    Point(True, maxPathLength)
                    if x == '0' else Point(False, -1)
                    for x in line.split()])

        # Distance of start point is 0
        self.point(self.start).dist = 0
        # Mark that start point's trace is not None, but is not other position
        self.point(self.start).trace = (-1, -1)

    # return the matrix element of graph at position pos
    def point(self, pos):
        return self.matrix[pos[0]][pos[1]]

    # true if pos is an empty point
    def validPos(self, pos):
        return 0 <= pos[0] < self.size and \
               0 <= pos[1] < self.size and \
               self.point(pos).valid

    # Heuristic function return Euclidean distance between pos and goal point
    def heuristic(self,pos):
        def EuclideanDistance(pos1,pos2):
            return math.sqrt((pos2[0]-pos1[0])**2+(pos2[1]-pos1[1])**2)
        
        return EuclideanDistance(pos,self.goal);
    
    # Heuristic function use my distance function
    # d = max(|x1-x2|,|y1-y2|)
    # Optimal heuristic, because it is the shortest path between 2 point 
    # when there are no obstacles block the path
    def myheuristic(self,pos):
        def MyDistance(pos1,pos2):
            return max(abs(pos1[0]-pos2[0]),abs(pos1[1]-pos2[1]))
        
        return MyDistance(pos,self.goal);
    
    # Path-estimation function f used in A* algorithm
    # f(pos) = g(pos) + h(pos)
    # g = distance from pos to start
    # h = heuristic distance from pos to goal
    def fAstar(self, pos):
        return self.point(pos).dist + self.heuristic(pos)

    # A* algorithm implementation
    def Astar(self):
        # Traversaling order to adjacent point, clockwise
        adjOrder = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]

        # Initialize priority queue with start point
        queue = []
        heap.heappush(queue, (0, self.start))
        distance = -1  # smallest distance from Start to Goal
        path = []  # list store points in the shortest path

        while queue:
            self.outputQueue(queue);
            d, p = heap.heappop(queue)

            if p == self.goal:
                distance = self.point(self.goal).dist

                while p != self.start:
                    path = [p] + path
                    p = self.point(p).trace
                path = [self.start] + path
                
                break;

            for adj in adjOrder:
                padj = (p[0] + adj[0], p[1] + adj[1])
                if self.validPos(padj) and self.fAstar(padj) > self.fAstar(p) + 1:
                    self.point(padj).dist = self.point(p).dist + 1
                    self.point(padj).trace = p
                    heap.heappush(queue, (self.point(padj).dist, padj))

        return distance, path

    
    def outputQueue(self,queue):
        m=[]
        for i in range(self.size):
            row=[]
            for j in range(self.size):
                if not(self.validPos((i,j))):
                    row.append('o')
                else:
                    row.append('-')
            m.append(row)
        m[self.start[0]][self.start[1]]='S'
        m[self.start[0]][self.start[1]]='S'
        for d,p in queue:
            m[p[0]][p[1]]='*'
        for row in m:
            for p in row:
                print(p,end=' ');
            print()
            
            
    # Output
    # Remember to replace output.txt to sys.argv[2]
    # sys.stdout=open('output.txt','w')
    def output(self, distance, path):
        print(distance)

        if distance > 0:
            for p in path:
                print(f'({p[0]},{p[1]})', end=' ')
            print()

            for i in range(self.size):
                for j in range(self.size):
                    if (i, j) == self.start:
                        print('S', end=' ')
                    elif (i, j) == self.goal:
                        print('G', end=' ')
                    elif (i, j) in path:
                        print('x', end=' ')
                    elif not (self.validPos((i, j))):
                        print('o', end=' ')
                    else:
                        print('-', end=' ')
                print();

class Form(Frame):
    """
    Class Form manage things show on Windows Form, include:
    - Menu: Allow to choose input file (lately development)
    - Label: pQueue, pos of current point, value of f(n) = g(n) + h(n)
    - Grid represent maps with:
        + Red cell is Start point
        + Flag cell is Goal point
        + Blue cell is in-process point
        + Orange cells is points on best current route from start to goal
        + Gray cells is valid point
        + Black cells is invalid point
    - Play button to Start process A* search
    """
    def __init__(self, parent, numRow, numCol):
        Frame.__init__(self, parent)

        self.fn = Label(self, text="0.0")
        self.gn = Label(self, text="0.0")
        self.hn = Label(self, text="800.0")

        self.parent = parent
        self.font_cell = tkFont.Font(family="monospace", size=20)
        self.lbl_pQueue = Label(self, text="pQueue")
        self.lbl_current_point = Label(self, text="Current Point: ")

        self.lbl_fn = Label(self, text="f(n)=")
        self.lbl_gn = Label(self, text="g(n)=")
        self.lbl_hn = Label(self, text="h(n)=")

        self.text_goal = "G"
        self.color_start = "red"
        self.color_current_point = "blue"
        self.color_route = "orange"
        self.color_block = "black"

        self.initUI(numRow, numCol)

    def initUI(self, num_row, num_col):
        self.parent.title("A* Heuristic")

        self.lbl_fn.grid(row=num_row, column=0)
        self.fn.grid(row=num_row, column=1)

        self.lbl_gn.grid(row=num_row + 1, column=0)
        self.gn.grid(row=num_row + 1, column=1)

        self.lbl_hn.grid(row=num_row + 2, column=0)
        self.hn.grid(row=num_row + 2, column=1)

        self.lbl_current_point.grid(row=num_row+3, column=0, columnspan=2)

        print(self.gn)

        for i in range(num_row):
            self.rowconfigure(i, pad=3)
        for i in range(num_col):
            self.columnconfigure(i, pad=3)

        labels = []
        color = "gray"

        for i in range(num_row):
            labels.append([])
            for j in range(num_col):
                labels[i].append(self.create_cell(i, j, color))

        self.pack()

    def create_cell(self, r, c, color="gray"):
        label1 = Label(self)
        label1.configure(width="2", background=color, font=self.font_cell)
        label1.grid(row=r, column=c)
        return label1

    def set_start_cell(self, start):
        self.create_cell(start[0], start[1], self.color_start)

    def set_goal_cell(self, goal):
        label_goal = self.create_cell(goal[0], goal[1])
        label_goal.configure(text=self.text_goal)

    def set_block_cell(self, block):
        self.create_cell(block[0], block[1], self.color_block)

    def load_cell_to_process(self, cell):
        self.create_cell(cell[0], cell[1], self.color_current_point)
        self.pack()

    def unload_cell(self, cell):
        self.create_cell(cell[0], cell[1])
        self.pack()

    def update_value_fgh(self, fn, gn, hn):
        self.fn['text'] = str(fn)
        self.gn['text'] = str(gn)
        self.hn['text'] = str(hn)
        self.pack()


g = Graph()
g.input()
distance, path = g.Astar()
g.output(distance, path)

# root = Tk()
# app = Form(root, 4, 9)
# app.set_start_cell([1, 1])
# app.set_goal_cell([3, 2])
# app.update_value_fgh(3.2, 2.9, 1.0)
# app.load_cell_to_process([1,2])
# root.mainloop()
