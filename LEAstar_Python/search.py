from heapq import heappop, heappush
import numpy as np
import time

INF = float('inf')

##################################################

def weighted(weight=1.):
    if weight == INF:
        return lambda g, h: h
    return lambda g, h: g + weight*h

uniform = weighted(0)
astar = weighted(1)
wastar2 = weighted(2)
wastar3 = weighted(2)
greedy = weighted(INF)
lexicographic = lambda g, h: (h, g)


def Astar(self):
    start, goal = self.samples[0], self.samples[1]
    start_tuple, goal_tuple = tuple(start), tuple(goal)
    came_from = {}
    g_cost = {}
    came_from[start_tuple] = None
    g_cost[start_tuple] = 0
    queue = [(self.MH_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex

    vertex_count, edge_count = 0, 0
    while queue:
        _, current_g, current = heappop(queue)
        current_tuple = tuple(current)
        if current_tuple == goal_tuple:
            print('Astar edge_count', edge_count, 'Astar vertex_count', vertex_count)
            return self.reconstruct_path(came_from), g_cost[goal_tuple]
        if current_tuple in g_cost and current_g > g_cost[current_tuple]:      # current_tuple is already expanded with a lower cost-to-come
            continue                                                           # queue can have repeated vertex with different cost
        vertex_count += 1
        for new in self.neighbor[current_tuple]:
            edge_count += 1
            if self.stspace._edge_fp(current, new):
                new_g = current_g + self.MH_dist(current, new)
                new_tuple = tuple(new)
                if new_tuple not in g_cost or new_g < g_cost[new_tuple]:
                    g_cost[new_tuple] = new_g
                    came_from[new_tuple] = current_tuple
                    heappush(queue, (new_g + self.MH_dist(new, goal), new_g, new))
    return None, None


def edge_Astar(self):
    start, goal = self.samples[0], self.samples[1]
    start_tuple, goal_tuple = tuple(start), tuple(goal)
    came_from = {}
    g_cost = {}
    came_from[start_tuple] = None
    g_cost[start_tuple] = 0
    edge_queue = []
    for neb in self.neighbor[start_tuple]:
        f_edge_cost = 0 + self.MH_dist(start, neb) + self.MH_dist(neb, goal)  # heuristic edge_cost
        heappush(edge_queue, (f_edge_cost, 0, start, neb))  # f_edge_cost, g_cost, source vertex, target vertex

    edge_count = 0
    while edge_queue:
        f_pop, g_pop, source, target = heappop(edge_queue)
        if goal_tuple in g_cost and f_pop >= g_cost[goal_tuple]:
            print('eAstar edge_count', edge_count)
            return self.reconstruct_path(came_from), g_cost[goal_tuple]
        source_tuple = tuple(source)
        '''
        if g_pop > g_cost[source_tuple]:   # not need for eA*
            continue
        '''
        edge_count += 1
        if self.stspace._edge_fp(source, target):
            target_g = g_pop + self.MH_dist(source, target)  # true edge_cost
            target_tuple = tuple(target)
            if target_tuple not in g_cost or target_g < g_cost[target_tuple]:
                g_cost[target_tuple] = target_g
                came_from[target_tuple] = source_tuple
                for neb in self.neighbor[target_tuple]:
                    neb_g_condidate = target_g + self.MH_dist(target, neb)  # heuristic edge_cost
                    if tuple(neb) in g_cost and neb_g_condidate >= g_cost[tuple(neb)]:  # edge is impossible to provide a better sol
                        continue                                                        # thus not added to queue
                    else:
                        f_edge_cost = neb_g_condidate + self.MH_dist(neb, goal)
                        heappush(edge_queue, (f_edge_cost, target_g, target, neb))
                    '''
                    # the following two lines are replaced by the above 6 lines
                    f_edge_cost = target_g + self.MH_dist(target, neb) + self.MH_dist(neb, goal)
                    heappush(edge_queue, (f_edge_cost, target_g, target, neb))
                    '''
    return None


