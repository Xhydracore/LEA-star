import numpy as np
import math, copy
import time, heapq
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
        # self.neighbor_dynamic = {}

        self.init_state = np.array([2,8])
        self.goal_state = np.array([18,18])

        self.edges_evaled = set()
        self.collision_edges = set()
        self.g_cost = {}  # LazySP_LPAstar
        self.rhs = {}
        self.queue_lpastar = []

        self.discret_res = 0.1  # discretize resolution for collision checking
        self.depth = 2  # LRAstar

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
        gamma = 30
        ner = min(4, gamma * (math.log10(N + 1) / N) ** (1 / 2))
        for i in range(N):
            near = []
            idx_flag = np.sum((abs(samples - samples[i]))**2, axis=1) <= 36 # ner**2
            for j in range(N):
                if idx_flag[j] and j != i:
                    near.append(samples[j])
            self.neighbor[samples_tuple[i]] = near
        if self.save_obs_samples:
            output = open('neighbor.pkl', 'wb')
            pickle.dump(self.neighbor, output)
            output.close()


    def LRAstar_LPAstar(self):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.init_state, self.goal_state
        self.start_tuple, self.goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        self.g_cost = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs[self.start_tuple] = 0
        self.queue_lpastar = [(0 + self.E_dist(start, goal), start)]  # LPAstar_Key, vertex

        edge_count, search_count = 0, 0
        collision_flag = True
        while True:
            if collision_flag:
                search_count += 1
                path_candidate = self.LRAstar_LPAstar_search()
                cost= self.g_cost[self.goal_tuple]
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag and path_candidate[-1] == self.goal_tuple:
                # print('LRAstar_LPAstar edge_count', edge_count, 'LRAstar_LPAstar search_count', search_count)
                # print(edge_count)
                return path_candidate, cost
            if e_select:
                edge_count += 1
                s, t = np.array(e_select[0]), np.array(e_select[1])
                edge_dist = self.E_dist(s, t)
                steps = int(np.ceil(edge_dist / self.discret_res))
                path = np.array([s + i * (t - s) / steps for i in range(steps)])
                collision_flag = self.world.check_path(path)
                if collision_flag:
                    self.collision_edges.add(e_select)
                    self.UpdateVertex(t)
                else:
                    for succ in self.neighbor[e_select[1]]:
                        self.UpdateVertex(succ)
                self.edges_evaled.add(e_select)
            else:
                collision_flag = True
        return None, None

    def LRAstar_LPAstar_search(self):
        Key_pop, current = heappop(self.queue_lpastar)
        Key_goal = self.CalculateKey(self.goal_state)
        while Key_pop <= Key_goal + 1e-9 or self.rhs[self.goal_tuple] != self.g_cost[self.goal_tuple]:
            current_tuple = tuple(current)
            if self.g_cost[current_tuple] > self.rhs[current_tuple]:
                self.g_cost[current_tuple] = self.rhs[current_tuple]
                flag, Path = self.Event_LPAstar(current)
                if flag:
                    return Path
                for succ in self.neighbor[current_tuple]:
                    self.UpdateVertex(succ)
            else:
                self.g_cost[current_tuple] = INF
                self.UpdateVertex(current)
                for succ in self.neighbor[current_tuple]:
                    self.UpdateVertex(succ)
            Key_pop, current = heappop(self.queue_lpastar)
            Key_goal = self.CalculateKey(self.goal_state)

    def Event_LPAstar(self, current):
        if tuple(current) == self.goal_tuple:
            flag = True
            Path = self.reconstruct_path_LPAstar(current)
            return flag, Path
        Path = self.reconstruct_path_LPAstar(current)
        depth_c = 0
        if len(self.edges_evaled) == 0:
            depth_c = len(Path) - 1
        else:
            for i in range(len(Path)-1):
                if (Path[i], Path[i + 1]) not in self.edges_evaled:
                    depth_c = depth_c + 1
        flag = False
        if depth_c >= self.depth:
            flag = True
        return flag, Path




    def LazySP_LPAstar(self):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        self.g_cost = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs[start_tuple] = 0
        self.queue_lpastar = [(0 + self.E_dist(start, goal), start)]  # LPAstar_Key, vertex

        edge_count, search_count = 0, 0
        collision_flag = True
        while True:
            if collision_flag:
                search_count += 1
                self.LazySP_LPAstar_search()
                cost = self.g_cost[goal_tuple]
                path_candidate = self.reconstruct_path_LPAstar(goal)

            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag:
                # print('LazySP_LPAstar edge_count', edge_count, 'LazySP_LPAstar search_count', search_count)
                # print(edge_count)
                return path_candidate, cost
            edge_count += 1
            s, t = np.array(e_select[0]), np.array(e_select[1])
            edge_dist = self.E_dist(s, t)
            steps = int(np.ceil(edge_dist / self.discret_res))
            path = np.array([s + i * (t - s) / steps for i in range(steps)])
            collision_flag = self.world.check_path(path)
            if collision_flag:
                self.collision_edges.add(e_select)
                self.UpdateVertex(t)
                # array = np.array(self.neighbor_dynamic[e_select[0]])
                # idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                # del self.neighbor_dynamic[e_select[0]][idx]
            self.edges_evaled.add(e_select)
        return None, None

    def CalculateKey(self, vertex):
        Key = min(self.g_cost[tuple(vertex)], self.rhs[tuple(vertex)]) + self.E_dist(vertex, self.goal_state)
        return Key

    def UpdateVertex(self, vertex):
        vertex_tuple = tuple(vertex)
        if vertex_tuple != tuple(self.init_state):
            cost = INF
            for pred in self.neighbor[vertex_tuple]:
                if (tuple(pred), vertex_tuple) in self.collision_edges:
                    continue
                cost_new = self.g_cost[tuple(pred)] + self.E_dist(pred, vertex)
                if cost_new < cost:
                    cost = cost_new
            self.rhs[vertex_tuple] = cost

        item_index = None
        for i, item in enumerate(self.queue_lpastar):
            if (vertex == item[1]).all():
                item_index = i
                break
        if item_index is not None:
            self.queue_lpastar[item_index] = self.queue_lpastar[-1]
            self.queue_lpastar.pop()
            # heapq.heapify(self.queue_lpastar)
            if item_index < len(self.queue_lpastar):
                heapq._siftup(self.queue_lpastar, item_index)
                heapq._siftdown(self.queue_lpastar, 0, item_index)

        if self.g_cost[vertex_tuple] != self.rhs[vertex_tuple]:
            heappush(self.queue_lpastar, (self.CalculateKey(vertex), vertex))

    def LazySP_LPAstar_search(self):
        Key_pop, current = heappop(self.queue_lpastar)
        Key_goal = self.CalculateKey(self.goal_state)
        while Key_pop <= Key_goal + 1e-9 or self.rhs[tuple(self.goal_state)] != self.g_cost[tuple(self.goal_state)]:
            current_tuple = tuple(current)
            if self.g_cost[current_tuple] > self.rhs[current_tuple]:
                self.g_cost[current_tuple] = self.rhs[current_tuple]
                for succ in self.neighbor[current_tuple]:
                    self.UpdateVertex(succ)
            else:
                self.g_cost[current_tuple] = INF
                self.UpdateVertex(current)
                for succ in self.neighbor[current_tuple]:
                    self.UpdateVertex(succ)
            Key_pop, current = heappop(self.queue_lpastar)
            Key_goal = self.CalculateKey(self.goal_state)

    def reconstruct_path_LPAstar(self, current):
        current_tuple = tuple(current)
        start_tuple = tuple(self.init_state)
        path = []
        while current_tuple != start_tuple:
            path.append(current_tuple)
            cost = INF
            for pred in self.neighbor[current_tuple]:
                if (tuple(pred), current_tuple) in self.collision_edges:
                    continue
                cost_new = self.g_cost[tuple(pred)] + self.E_dist(pred, current)
                if cost_new < cost:
                    cost = cost_new
                    next = pred
            current = next
            current_tuple = tuple(current)
        path.append(start_tuple)
        path.reverse()
        self.path = path
        return path


    def LRAstar(self, weight=1):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        edge_count, search_count = 0, 0
        collision_flag = True
        while True:
            if collision_flag:
                search_count += 1
                path_candidate, cost = self.LRAstar_search(start, goal, start_tuple, goal_tuple, weight)
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag and path_candidate[-1] == goal_tuple:
                # print('LRAstar edge_count', edge_count, 'LRAstar search_count', search_count)
                # print(edge_count, search_count)
                return path_candidate, cost
            if e_select:
                edge_count += 1
                s, t = np.array(e_select[0]), np.array(e_select[1])
                edge_dist = self.E_dist(s, t)
                steps = int(np.ceil(edge_dist / self.discret_res))
                path = np.array([s + i * (t - s) / steps for i in range(steps)])
                collision_flag = self.world.check_path(path)
                if collision_flag:
                    self.collision_edges.add(e_select)
                    # array = np.array(self.neighbor_dynamic[e_select[0]])
                    # idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                    # del self.neighbor_dynamic[e_select[0]][idx]
                self.edges_evaled.add(e_select)
            else:
                collision_flag = True
        return None, None

    def LRAstar_search(self, start, goal, start_tuple, goal_tuple, weight):
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}

        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            flag, Path = self.Event(current, came_from)
            if flag:
                if goal_tuple in g_cost:
                    return Path, g_cost[goal_tuple]
                else:
                    return Path, INF

            if current_tuple in g_cost and current_g > g_cost[current_tuple]:  # current_tuple is already expanded with a lower cost-to-come
                continue  # queue can have repeated vertex with different cost
            for new in self.neighbor[current_tuple]:
                new_tuple = tuple(new)
                if (current_tuple, new_tuple) in self.collision_edges:
                    continue
                new_g = current_g + self.E_dist(current, new)
                if new_tuple not in g_cost or new_g < g_cost[new_tuple]:
                    g_cost[new_tuple] = new_g
                    came_from[new_tuple] = current_tuple
                    heappush(queue, (new_g + weight * self.E_dist(new, goal), new_g, new))
        return None, None

    def Event(self, current, came_from):
        if tuple(current) == tuple(self.goal_state):
            flag = True
            Path = self.reconstruct_path(came_from, current)
            return flag, Path
        Path = self.reconstruct_path(came_from, current)
        depth_c = 0
        if len(self.edges_evaled) == 0:
            depth_c = len(Path) - 1
        else:
            for i in range(len(Path)-1):
                if (Path[i], Path[i + 1]) not in self.edges_evaled:
                    depth_c = depth_c + 1
        flag = False
        if depth_c >= self.depth:
            flag = True
        return flag, Path


    def LazySP(self, weight=1):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        edge_count, search_count = 0, 0
        collision_flag = True
        while True:
            if collision_flag:
                search_count += 1
                path_candidate, cost = self.LazySP_Astar(start, goal, start_tuple, goal_tuple, weight)
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag:
                # print('LazySP edge_count', edge_count, 'LazySP search_count', search_count)
                return path_candidate, cost
            edge_count += 1
            s, t = np.array(e_select[0]), np.array(e_select[1])
            edge_dist = self.E_dist(s, t)
            steps = int(np.ceil(edge_dist / self.discret_res))
            path = np.array([s + i * (t - s) / steps for i in range(steps)])
            collision_flag = self.world.check_path(path)
            if collision_flag:
                self.collision_edges.add(e_select)
                # array = np.array(self.neighbor_dynamic[e_select[0]])
                # idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                # del self.neighbor_dynamic[e_select[0]][idx]
            self.edges_evaled.add(e_select)
        return None, None

    def LazySP_Astar(self, start, goal, start_tuple, goal_tuple, weight):
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}

        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            if current_tuple == goal_tuple:
                return self.reconstruct_path(came_from, goal), g_cost[goal_tuple]
            if current_tuple in g_cost and current_g > g_cost[
                current_tuple]:  # current_tuple is already expanded with a lower cost-to-come
                continue  # queue can have repeated vertex with different cost
            for new in self.neighbor[current_tuple]:
                new_tuple = tuple(new)
                if (current_tuple, new_tuple) in self.collision_edges:
                    continue
                new_g = current_g + self.E_dist(current, new)
                if new_tuple not in g_cost or new_g < g_cost[new_tuple]:
                    g_cost[new_tuple] = new_g
                    came_from[new_tuple] = current_tuple
                    heappush(queue, (new_g + weight * self.E_dist(new, goal), new_g, new))
        return None, None

    def Path_is_evaluated(self, path_candidate):
        solution_flag = 1
        e_select = None
        for i in range(len(path_candidate) - 1):
            if (path_candidate[i], path_candidate[i + 1]) not in self.edges_evaled:
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
                # print('Astar edge_count', edge_count, 'Astar vertex_count', vertex_count)
                return self.reconstruct_path(came_from, goal), g_cost[goal_tuple]
            if current_tuple in g_cost and current_g > g_cost[
                current_tuple]:  # current_tuple is already expanded with a lower cost-to-come
                continue  # queue can have repeated vertex with different cost
            vertex_count += 1
            for new in self.neighbor[current_tuple]:
                edge_count += 1
                edge_dist = self.E_dist(current, new)
                steps = int(np.ceil(edge_dist / self.discret_res))
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
                # print('eAstar edge_count', edge_count)
                return self.reconstruct_path(came_from, goal), g_cost[goal_tuple]
            source_tuple = tuple(source)
            target_tuple = tuple(target)
            edge_dist = self.E_dist(source, target)  # heuristic edge_cost
            target_g = g_pop + edge_dist
            if target_tuple in g_cost and g_cost[target_tuple] <= target_g:
                continue
            edge_count += 1
            steps = int(np.ceil(edge_dist / self.discret_res))
            path = np.array([source + i * (target - source) / steps for i in range(steps)])
            if not self.world.check_path(path):
                if target_tuple not in g_cost or target_g < g_cost[target_tuple]:  # true edge_cost after collision checking
                    g_cost[target_tuple] = target_g
                    came_from[target_tuple] = source_tuple
                    if target_tuple == goal_tuple:
                        continue
                    for neb in self.neighbor[target_tuple]:
                        neb_g_condidate = target_g + self.E_dist(target, neb)  # heuristic edge_cost
                        if tuple(neb) in g_cost and neb_g_condidate >= g_cost[tuple(neb)]:  # edge is impossible to provide a better sol
                            continue  # thus not added to queue
                        f_edge_cost = neb_g_condidate + weight * self.E_dist(neb, goal)
                        heappush(edge_queue, (f_edge_cost, target_g, target, neb))
        return None, None

    def LWAstar(self, weight=1):
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}
        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex
        edge_queue = []

        edge_count = 0
        while queue or edge_queue:
            if queue:
                minf_v, _, _ = queue[0]  # access the smallest item without popping
            else:
                minf_v = INF     # empty set
            if edge_queue:
                minf_e, _, _, _ = edge_queue[0]
            else:
                minf_e = INF
            if goal_tuple in g_cost and g_cost[goal_tuple] <= min(minf_v, minf_e):
                # print('eAstar edge_count', edge_count)
                # print(edge_count)
                return self.reconstruct_path(came_from, goal), g_cost[goal_tuple]
            if minf_v <= minf_e:
                _, current_g, current = heappop(queue)
                current_tuple = tuple(current)
                for neb in self.neighbor[current_tuple]:
                    f_edge_cost = current_g + self.E_dist(current, neb) + weight * self.E_dist(neb, goal) # heuristic edge_cost
                    heappush(edge_queue, (f_edge_cost, current_g, current, neb))
            else:
                f_pop, g_pop, source, target = heappop(edge_queue)
                source_tuple = tuple(source)
                target_tuple = tuple(target)
                edge_dist = self.E_dist(source, target)  # heuristic edge_cost
                target_g = g_pop + edge_dist
                if target_tuple in g_cost and g_cost[target_tuple] <= target_g:
                    continue
                edge_count += 1
                steps = int(np.ceil(edge_dist / self.discret_res))
                path = np.array([source + i * (target - source) / steps for i in range(steps)])
                if not self.world.check_path(path):
                    if target_tuple not in g_cost or target_g < g_cost[target_tuple]:  # true edge_cost after collision checking
                        g_cost[target_tuple] = target_g
                        came_from[target_tuple] = source_tuple
                        heappush(queue, (target_g + weight * self.E_dist(target, goal), target_g, target))
        return None, None

    def reconstruct_path(self, came_from, current):
        current = tuple(current)
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

    t0 = time.perf_counter()
    for i in range(1):
        path, cost = GS.Astar(weight)
    print('Astar time: ', time.perf_counter() - t0, 'Astar cost', cost)
    t0 = time.perf_counter()
    for i in range(1):
        path, cost = GS.edge_Astar(weight)
    print('eAstar time: ', time.perf_counter() - t0, 'eAstar cost', cost)

    t0 = time.perf_counter()
    for i in range(1):
        path, cost = GS.LWAstar(weight)
    print('LWAstar time: ', time.perf_counter() - t0, 'LWAstar cost', cost)
    t0 = time.perf_counter()
    for i in range(1):
        path, cost = GS.LazySP(weight)
    print('LazySP time: ', time.perf_counter() - t0, 'LazySP cost', cost)

    t0 = time.perf_counter()
    for i in range(1):
        path, cost = GS.LazySP_LPAstar()
    print('LazySP_LPAstar time: ', time.perf_counter() - t0, 'LazySP_LPAstar cost', cost)
    
    t0 = time.perf_counter()
    for i in range(1):
        path, cost = GS.LRAstar()
    print('LRAstar time: ', time.perf_counter() - t0, 'LRAstar cost', cost)
    
    t0 = time.perf_counter()
    for i in range(1):
        path, cost = GS.LRAstar_LPAstar()
    print('LRAstar_LPAstar time: ', time.perf_counter() - t0, 'LRAstar_LPAstar cost', cost)

