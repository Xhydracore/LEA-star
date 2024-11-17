import numpy as np
import math
from time import sleep, time
from heapq import heappop, heappush
import matplotlib.pyplot as plt
from scipy.io import loadmat
import pickle
from create_world import World
INF = float('inf')


class Search_Alg():
    def __init__(self, world, save_obs_samples=False) -> None:

        self.world = world
        self.save_obs_samples = save_obs_samples
        self.neighbor = {}
        self.neighbor_current = {}

        self.init_state = [2,2]
        self.goal_state = [18,18]


    def MH_dist(self, p, q):
        return sum(abs(p - q))

    def E_dist(self, p, q):
        diff = abs(p - q)
        return np.sqrt(sum(diff ** 2))

    def sampling(self, n, samples=[]):
        if len(samples) < 1:
            samples = np.random.uniform([0, 0], [20, 20], size=(n, 2))
            samples = np.append([self.init_state, self.goal_state], samples, axis=0)
        samples_tuple = []
        for i in range(len(samples)):
            samples_tuple.append(tuple(samples[i]))
        self.samples, self.samples_tuple = samples, samples_tuple
        if self.save_obs_samples:
            output = open('samples.pkl', 'wb')
            pickle.dump(self.samples, output)
            pickle.dump(self.samples_tuple, output)
            output.close()

    def find_neighbor(self):
        samples, samples_tuple = self.samples, self.samples_tuple
        N = len(samples)
        for i in range(N):
            near = []
            debug = np.sum((abs(samples - samples[i]))**2, axis=1)
            idx_flag = np.sum((abs(samples - samples[i]))**2, axis=1) < 36
            for j in range(N):
                if idx_flag[j] and j != i:
                    near.append(samples[j])
            self.neighbor[samples_tuple[i]] = near
        if self.save_obs_samples:
            output = open('neighbor.pkl', 'wb')
            pickle.dump(self.neighbor, output)
            output.close()


    def LazySP(self, weight=1):
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        edges_evaled = set()
        vertex_fail = set()
        edge_count, search_count = 0, 0
        collision_flag = True
        while True:
            if collision_flag:
                search_count += 1
                path_candidate, cost = self.LazySP_Astar(start, goal, start_tuple, goal_tuple, vertex_fail, weight)
            solution_flag, e_select = self.LazySP_path_evaluated(path_candidate, edges_evaled)
            if solution_flag:
                print('LazySP edge_count', edge_count, 'LazySP search_count', search_count)
                return path_candidate, cost
            edge_count += 1
            edge_dist = self.E_dist(np.array(e_select[1]), np.array(e_select[0]))
            steps = int(np.ceil(edge_dist / 0.1))
            path = np.array([np.array(e_select[0]) + i * (np.array(e_select[1]) - np.array(e_select[0])) / steps for i in range(steps)])
            collision_flag = self.world.check_point(path[-1])
            if collision_flag:
                vertex_fail.add(e_select[1])
            else:
                collision_flag = self.world.check_path(path)
            if collision_flag:
                array = np.array(self.neighbor[e_select[0]])
                idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                del self.neighbor[e_select[0]][idx]
            edges_evaled.add(e_select)
        return None, None

    def LazySP_Astar(self, start, goal, start_tuple, goal_tuple, vertex_fail, weight):
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}

        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            if current_tuple == goal_tuple:
                return self.reconstruct_path(came_from), g_cost[goal_tuple]
            if current_tuple in g_cost and current_g > g_cost[
                current_tuple]:  # current_tuple is already expanded with a lower cost-to-come
                continue  # queue can have repeated vertex with different cost
            for new in self.neighbor[current_tuple]:
                new_tuple = tuple(new)
                if new_tuple not in vertex_fail:
                    new_g = current_g + self.E_dist(current, new)
                    if new_tuple not in g_cost or new_g < g_cost[new_tuple]:
                        g_cost[new_tuple] = new_g
                        came_from[new_tuple] = current_tuple
                        heappush(queue, (new_g + weight * self.E_dist(new, goal), new_g, new))
        return None, None

    def LazySP_path_evaluated(self, path_candidate, edges_evaled):
        solution_flag = 1
        e_select = None
        for i in range(len(path_candidate) - 1):
            if (path_candidate[i], path_candidate[i + 1]) not in edges_evaled:
                solution_flag = 0
                e_select = (path_candidate[i], path_candidate[i + 1])
                return solution_flag, e_select
        return solution_flag, e_select

    def Astar(self, weight=1):
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}
        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex

        vertex_count, edge_count = 0, 0
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            if current_tuple == goal_tuple:
                print('Astar edge_count', edge_count, 'Astar vertex_count', vertex_count)
                return self.reconstruct_path(came_from), g_cost[goal_tuple]
            if current_tuple in g_cost and current_g > g_cost[
                current_tuple]:  # current_tuple is already expanded with a lower cost-to-come
                continue  # queue can have repeated vertex with different cost
            vertex_count += 1
            for new in self.neighbor[current_tuple]:
                edge_count += 1
                edge_dist = self.E_dist(current, new)
                steps = int(np.ceil(edge_dist / 0.1))
                path = np.array([current + i * (new - current) / steps for i in range(steps)])
                if not self.world.check_path(path):
                    new_g = current_g + edge_dist
                    new_tuple = tuple(new)
                    if new_tuple not in g_cost or new_g < g_cost[new_tuple]:
                        g_cost[new_tuple] = new_g
                        came_from[new_tuple] = current_tuple
                        heappush(queue, (new_g + weight * self.E_dist(new, goal), new_g, new))
        return None, None

    def edge_Astar(self, weight=1):
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}
        edge_queue = []
        for neb in self.neighbor[start_tuple]:
            f_edge_cost = 0 + self.E_dist(start, neb) + weight * self.E_dist(neb, goal)  # heuristic edge_cost
            heappush(edge_queue, (f_edge_cost, 0, start, neb))  # f_edge_cost, g_cost, source vertex, target vertex

        edge_count = 0
        while edge_queue:
            f_pop, g_pop, source, target = heappop(edge_queue)
            if goal_tuple in g_cost and f_pop >= g_cost[goal_tuple]:
                print('eAstar edge_count', edge_count)
                return self.reconstruct_path(came_from), g_cost[goal_tuple]
            source_tuple = tuple(source)
            target_tuple = tuple(target)
            edge_dist = self.E_dist(source, target)  # heuristic edge_cost
            target_g = g_pop + edge_dist
            if target_tuple in g_cost and g_cost[target_tuple] <= target_g:
                continue
            edge_count += 1
            steps = int(np.ceil(edge_dist / 0.1))
            path = np.array([source + i * (target - source) / steps for i in range(steps)])
            if not self.world.check_path(path):
                if target_tuple not in g_cost or target_g < g_cost[target_tuple]:  # true edge_cost after collision checking
                    g_cost[target_tuple] = target_g
                    came_from[target_tuple] = source_tuple
                    for neb in self.neighbor[target_tuple]:
                        neb_g_condidate = target_g + self.E_dist(target, neb)  # heuristic edge_cost
                        if tuple(neb) in g_cost and neb_g_condidate >= g_cost[
                            tuple(neb)]:  # edge is impossible to provide a better sol
                            continue  # thus not added to queue
                        f_edge_cost = neb_g_condidate + weight * self.E_dist(neb, goal)
                        heappush(edge_queue, (f_edge_cost, target_g, target, neb))
        return None

    def reconstruct_path(self, came_from):
        current = self.samples_tuple[1]
        start = self.samples_tuple[0]
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        self.path = path
        return path


if __name__ == "__main__":

    graphdata = loadmat('GraphPY1.mat')
    Vertices = graphdata['Vertices']
    # Vertices = np.round(Vertices, 3)
    world_obs = graphdata['world_obs']
    world_obs_map = graphdata['world_obs_map']

    world = World(0, 0, 20, 20)
    world.obs_map = world_obs_map
    world.obs = world_obs

    GS = Search_Alg(world, save_obs_samples=False)
    GS.sampling(3000, Vertices)
    GS.find_neighbor()

    weight = 1
    t0 = time()
    for i in range(1):
        path, cost = GS.Astar(weight)
    print('Astar time: ', time() - t0, 'Astar cost', cost)
    t0 = time()
    for i in range(1):
        path, cost = GS.edge_Astar(weight)
    print('eAstar time: ', time() - t0, 'eAstar cost', cost)
    t0 = time()
    for i in range(1):
        path, cost = GS.LazySP(weight)
        print('LazySP time: ', time() - t0, 'LazySP cost', cost)