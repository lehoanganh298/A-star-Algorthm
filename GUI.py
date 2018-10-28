import heapq as heap
import math
from tkinter import Tk, CENTER
from tkinter.ttk import Label, Frame
import tkinter.font as tkFont

UPDATE_CLOCK = 700
# Traversaling order to adjacent get_cell, clockwise
ADJ_ORDER = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]


class Point:
    """ store information of each get_cell in the grid space """

    def __init__(self, valid, dist, trace=None):
        self.valid = valid  # true=empty, false=obstacle
        self.dist = dist  # distance from the start get_cell
        self.trace = trace  # store previous get_cell in the path to the goal


class App:
    """
    Class App manage things show on Windows Form, include:
    - Form: Manage window form
    - Menu: Allow to choose input file (lately development)
    - Label: pQueue, pos of current get_cell, value of f(n) = g(n) + h(n)
    - Grid represent maps with:
        + Red cell is Start get_cell
        + Flag cell is Goal get_cell
        + Blue cell is in-process get_cell
        + Orange cells is points on best current route from start to goal
        + Gray cells is valid get_cell
        + Black cells is invalid get_cell
    - Play button to Start process A* search
    """

    def __init__(self):

        self.queue = []
        self.distance = -1
        self.path = []

        self.size = 0
        self.start = tuple()
        self.goal = tuple()
        self.matrix = []
        self.labels = None

        self.form = Tk()
        self.form.title('A* Search')

        self.font_cell = tkFont.Font(family="monospace", size=20)

        self.lbl_fn = Label(self.form, text="f(n) = 0.0", justify=CENTER)
        self.lbl_gn = Label(self.form, text="g(n) = 0.0", justify=CENTER)
        self.lbl_hn = Label(self.form, text="h(n) = 0.0", justify=CENTER)
        self.lbl_distance = Label(self.form, text="Distance: ", justify=CENTER)

        self.color_goal = "blue"
        self.color_start = "red"
        self.color_current_point = "white"
        self.color_visited_point = "orange"
        self.color_heap = "green"
        self.color_route = "blue"
        self.color_block = "black"

        self.input()
        heap.heappush(self.queue, (0, self.start))
        self.init_ui()
        self.updater()

    # Read input
    # Remember to replace input.txt to sys.argv[1]
    def input(self):
        with open('input.txt') as f:
            self.size = int(next(f))
            self.start = tuple([int(x) for x in next(f).split()])
            self.goal = tuple([int(x) for x in next(f).split()])

            # self.matrix = []
            max_path_length = self.size * self.size  # path from start to goal can not exceed size^2

            # Read the matrix
            # 0 -> Point(True,maxPathLength,None)
            # 1 -> Point(False,-1,None)
            for line in f:
                self.matrix.append([
                    Point(True, max_path_length)
                    if x == '0' else Point(False, -1)
                    for x in line.split()])

        # Distance of start get_cell is 0
        self.get_cell(self.start).dist = 0
        # Mark that start get_cell's trace is not None, but is not other position
        self.get_cell(self.start).trace = (-1, -1)

    # return the matrix element of graph at position pos
    def get_cell(self, pos):
        return self.matrix[pos[0]][pos[1]]

    def get_label(self, pos):
        return self.labels[pos[0]][pos[1]]

    def init_ui(self):
        self.labels = []
        for i in range(self.size):
            self.labels.append([])
            for j in range(self.size):
                if self.matrix[i][j].valid:
                    self.labels[i].append(self.create_cell(i, j))
                else:
                    self.labels[i].append(self.create_cell(i, j, self.color_block))

        self.get_label(self.start).configure(background=self.color_start, text="S")
        self.get_label(self.goal).configure(background=self.color_goal, text="G")

        self.lbl_fn.grid(row=self.size, columnspan=self.size)
        self.lbl_gn.grid(row=self.size + 1, columnspan=self.size)
        self.lbl_hn.grid(row=self.size + 2, columnspan=self.size)
        self.lbl_distance.grid(row=self.size + 3, columnspan=self.size)

        for i in range(self.size):
            self.form.rowconfigure(i, pad=3)
            self.form.columnconfigure(i, pad=3)

    def create_cell(self, r, c, color="gray"):
        label1 = Label(self.form)
        label1.configure(width="2", background=color, font=self.font_cell)
        label1.grid(row=r, column=c)
        return label1

    def run(self):
        self.form.mainloop()

    # Heuristic function return Euclidean distance between pos and goal get_cell
    def heuristic(self, pos):
        def euclidean_distance(pos1, pos2):
            return math.sqrt((pos[0] - pos1[0]) ** 2 + (pos2[1] - pos1[1]) ** 2)
        return euclidean_distance(pos, self.goal)

    # path-estimation function f used in A* algorithm
    # f(pos) = g(pos) + h(pos)
    # g = distance from pos to start
    # h = heuristic distance from pos to goal
    def f_astar(self, pos):
        return self.get_cell(pos).dist + self.heuristic(pos)

        # true if pos is an empty get_cell

    def valid_pos(self, pos):
        return 0 <= pos[0] < self.size and \
               0 <= pos[1] < self.size and \
               self.get_cell(pos).valid

    def next_step(self):
        if self.queue:
            d, p = heap.heappop(self.queue)

            if p == self.goal:
                self.distance = self.get_cell(self.goal).dist

                while p != self.start:
                    self.queue = []
                    self.path = [p] + self.path
                    self.get_label(p).configure(background=self.color_route)
                    p = self.get_cell(p).trace

                self.path = [self.start] + self.path

            else:
                if p != self.start:
                    self.get_label(p).configure(background=self.color_visited_point)

                for adj in ADJ_ORDER:
                    padj = (p[0] + adj[0], p[1] + adj[1])
                    if self.valid_pos(padj) and self.f_astar(padj) > self.f_astar(p) + 1:
                        self.get_cell(padj).dist = self.get_cell(p).dist + 1
                        self.get_cell(padj).trace = p
                        self.get_label(padj).configure(background=self.color_heap)
                        heap.heappush(self.queue, (self.get_cell(padj).dist, padj))

        # self.form.after(UPDATE_CLOCK, self.next_step())

    def updater(self):
        self.next_step()
        self.form.after(UPDATE_CLOCK, self.updater)


app = App()
app.run()
