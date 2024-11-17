from time import time
from enum import auto
from copy import deepcopy


class Timer:
    VERTEX_CHECK = 0
    EDGE_CHECK = 4
    SAMPLE = 1
    PLAN = 2
    CREATE = 3
    FORWARD = 5
    NN = 6
    EXPAND = 7
    HEAP = 8
    GPU = 9
    SHORTEST_PATH = 10
    
    def __init__(self):
        self.log = []
    
    def start(self):
        pass
        # self.st = time()
        
    def finish(self, action):
        pass
        # self.log.append([float(self.st), time(), action])