from kuka_env_bench import KukaEnv
import pybullet as p
import numpy as np
import math, random
import time, heapq
from heapq import heappop, heappush
import matplotlib.pyplot as plt
import pickle
import scipy.io
INF = float('inf')

class GymEnvironment():
    def __init__(self, statespace, encoder_model = None) -> None:

        self.stspace = statespace

        self.obstacle_idx = []
        self.obstacles = []
        self.neighbor = {}

        self.edges_evaled = set()
        self.collision_edges = set()
        self.g_cost = {}  # LazySP_LPAstar
        self.rhs = {}
        self.queue_lpastar = []
        self.vertex_count = 0

        self.depth = 2  # LRAstar

        self.dim = self.stspace.config_dim 
        self.bound_norm = np.linalg.norm(self.stspace.bound[self.dim:] - self.stspace.bound[:self.dim])
    
    def reset(self, n, new_obstacle=True):
        """Resets the Environment
        """

        ### environment change
        if new_obstacle:
            for idx in self.obstacle_idx:
                self.stspace.remove_body(idx)
            self.obstacle_idx.clear()
            self.populateRandomObstacles(n)
    
    def populateRandomObstacles(self, n):
        """Creates n random obstacles boxes in the environment
        """
        tableTop = True
        self.obstacles.clear()
        self.obstacle_idx.clear()
        lb = 0.3
        hb = 0.7

        if tableTop:
            self._populateRandomObsTableTop(n)
        else:
            for _ in range(n):
                position = np.empty(3)
                position[2] = np.random.uniform(low=-0.1, high=hb+0.15) # higher z
                xy = np.random.uniform(low=-hb, high=hb, size=(2,))
                while np.linalg.norm(xy) < lb or np.linalg.norm(xy) > hb:
                    xy = np.random.uniform(low=-hb, high=hb, size=(2,))
                position[:2] = xy

                half_extents = np.array([.1, .1, .1])
                self.obstacles.append((half_extents, position))
        self.obs_visual()
    
    def _populateRandomObsTableTop(self, n):
        """Creates obstacles for table top experiment
        """
        ymin = 0.2
        lb = 0.4
        hb = 0.6
        hb_lim = hb + .1
        i = 0
        while i < n:
            position = np.empty(3)
            position[:2] = np.random.uniform(low=[-hb, ymin], high=[hb, hb_lim])
            while np.linalg.norm(position[:2]) < lb or np.linalg.norm(position[:2]) > hb_lim:
                position[:2] = np.random.uniform(low=[-hb, ymin], high=[hb, hb_lim])
            half_extents = np.random.uniform(low=.025, high=.07, size=(3,))

            new_obs = (half_extents, position)

            half_extents[2] = np.random.uniform(low=0.075, high=0.3)
            position[2] = half_extents[2] + .025 # to make objects sit on table
            self.obstacles.append((half_extents, position))
            i += 1
        
        # prevent `cheating` paths going around
        self.obstacles.append(([0.75, 0.8, .01],[0, 0.49, 1.19]))   # top wall obstacle
        self.obstacles.append(([0.75, .01, .6],[0, -.3, .6]))     # back wall obstacle

        # self.obstacles.append(([0.75, 0.8, .01], [0, 0.49, 0.99]))  # top wall obstacle
        # self.obstacles.append(([0.75, .01, 0.5], [0, -.3, .5]))  # back wall obstacle

    def obs_visual(self):
        for halfExtents, basePosition in self.obstacles:
            body_idx = self.stspace.create_voxel(halfExtents, basePosition)
            self.obstacle_idx.append(body_idx)

    def MH_dist(self, p, q):
        return sum(abs(p - q))

    def E_dist(self, p, q):
        diff = abs(p - q)
        return np.sqrt(sum(diff ** 2))

    def sampling(self, n):
        samples = self.stspace.sample_n_points(n)
        samples_tuple = []
        for i in range(len(samples)):
            samples_tuple.append(tuple(samples[i]))
        self.samples, self.samples_tuple = samples, samples_tuple

    def getStartGoal(self):
        dist = 0
        self.init_state, self.goal_state = None, None
        while dist < 3:
            start_idx = random.randint(0, len(self.samples) - 1)
            if self.stspace._state_fp(self.samples[start_idx]):
                self.init_state = self.samples[start_idx]
            goal_idx = random.randint(0, len(self.samples)-1)
            if self.stspace._state_fp(self.samples[goal_idx]):
                self.goal_state = self.samples[goal_idx]
            if self.init_state is not None and self.goal_state is not None:
                dist = self.E_dist(self.init_state, self.goal_state)

    def find_neighbor(self):
        self.neighbor = {}
        samples, samples_tuple = self.samples, self.samples_tuple
        N = len(samples)
        gamma = 7
        ner = min(3, gamma * (math.log10(N+1)/N)**(1/7))
        for i in range(N):
            near = []
            idx_flag = np.sum((abs(samples - samples[i]))**2, axis=1) < ner**2
            for j in range(N):
                if idx_flag[j] and j != i:
                    near.append(samples[j])
            self.neighbor[samples_tuple[i]] = near

    def LRAstar_LPAstar(self):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.init_state, self.goal_state
        self.start_tuple, self.goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        self.vertex_fail = set()
        self.g_cost = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs[self.start_tuple] = 0
        self.queue_lpastar = [(0 + self.E_dist(start, goal), start)]  # LPAstar_Key, vertex

        edge_count, self.vertex_count, search_count = 0, 0, 0
        free_flag = False
        while True:
            if not free_flag:
                search_count += 1
                path_candidate = self.LRAstar_LPAstar_search()
                cost = self.g_cost[self.goal_tuple]
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag and path_candidate[-1] == self.goal_tuple:
                # print('LRAstar_LPAstar edge_count', edge_count, 'LRAstar_LPAstar search_count', search_count)
                # print(edge_count)
                return edge_count, self.vertex_count, cost
            if e_select:
                edge_count += 1
                s, t = np.array(e_select[0]), np.array(e_select[1])
                free_flag = self.stspace._state_fp(t)
                if not free_flag:
                    self.vertex_fail.add(e_select[1])
                else:
                    free_flag = self.stspace._edge_fp(s, t)
                if not free_flag:
                    self.collision_edges.add(e_select)
                    self.UpdateVertex(t)
                else:
                    for succ in self.neighbor[e_select[1]]:
                        self.UpdateVertex(succ)
                self.edges_evaled.add(e_select)
            else:
                free_flag = False
        return None, None, None

    def LRAstar_LPAstar_search(self):
        Key_pop, current = heappop(self.queue_lpastar)
        Key_goal = self.CalculateKey(self.goal_state)
        while Key_pop <= Key_goal + 1e-9 or self.rhs[self.goal_tuple] != self.g_cost[self.goal_tuple]:
            current_tuple = tuple(current)
            self.vertex_count += 1
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
        start, goal = self.init_state, self.goal_state
        self.start_tuple, self.goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        self.vertex_fail = set()
        self.g_cost = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs = {sample_tuple: INF for sample_tuple in self.samples_tuple}
        self.rhs[self.start_tuple] = 0
        self.queue_lpastar = [(0 + self.E_dist(start, goal), start)]  # LPAstar_Key, vertex

        edge_count, self.vertex_count, search_count = 0, 0, 0
        free_flag = False
        while True:
            if not free_flag:
                search_count += 1
                self.LazySP_LPAstar_search()
                cost = self.g_cost[self.goal_tuple]
                path_candidate = self.reconstruct_path_LPAstar(goal)
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag:
                # print('LazySP_LPAstar edge_count', edge_count, 'LazySP_LPAstar search_count', search_count)
                # print(edge_count)
                return edge_count, self.vertex_count, cost
            edge_count += 1
            s, t = np.array(e_select[0]), np.array(e_select[1])

            free_flag = self.stspace._state_fp(t)
            if not free_flag:
                self.vertex_fail.add(e_select[1])
            else:
                free_flag = self.stspace._edge_fp(s, t)
            if not free_flag:
                self.collision_edges.add(e_select)
                self.UpdateVertex(t)
                # array = np.array(self.neighbor_dynamic[e_select[0]])
                # idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                # del self.neighbor_dynamic[e_select[0]][idx]
            self.edges_evaled.add(e_select)
        return None, None, None

    def CalculateKey(self, vertex):
        Key = min(self.g_cost[tuple(vertex)], self.rhs[tuple(vertex)]) + self.E_dist(vertex, self.goal_state)
        return Key

    def UpdateVertex(self, vertex):
        vertex_tuple = tuple(vertex)
        if vertex_tuple in self.vertex_fail:
            self.rhs[vertex_tuple] = INF
        else:
            if vertex_tuple != self.start_tuple:
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
        while Key_pop <= Key_goal + 1e-9 or self.rhs[self.goal_tuple] != self.g_cost[self.goal_tuple]:
            current_tuple = tuple(current)
            self.vertex_count += 1
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
        path = []
        while current_tuple != self.start_tuple:
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
        path.append(self.start_tuple)
        path.reverse()
        self.path = path
        return path


    def LRAstar(self, depth, weight=1):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.depth = depth
        self.edges_evaled = set()
        self.collision_edges = set()
        self.vertex_fail = set()
        edge_count, self.vertex_count, search_count = 0, 0, 0
        free_flag = False
        while True:
            if not free_flag:
                search_count += 1
                path_candidate, cost = self.LRAstar_search(start, goal, start_tuple, goal_tuple, weight)
                if path_candidate is None:
                    return None, None, None
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag and path_candidate[-1] == goal_tuple:
                # print('LRAstar edge_count', edge_count, 'LRAstar search_count', search_count)
                # print(edge_count, search_count)
                return edge_count, self.vertex_count, cost
            if e_select:
                edge_count += 1
                s, t = np.array(e_select[0]), np.array(e_select[1])

                free_flag = self.stspace._state_fp(t)
                if not free_flag:
                    self.vertex_fail.add(e_select[1])
                else:
                    free_flag = self.stspace._edge_fp(s, t)
                if not free_flag:
                    self.collision_edges.add(e_select)
                    # array = np.array(self.neighbor_dynamic[e_select[0]])
                    # idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                    # del self.neighbor_dynamic[e_select[0]][idx]
                self.edges_evaled.add(e_select)
            else:
                free_flag = False

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
            self.vertex_count += 1
            for new in self.neighbor[current_tuple]:
                new_tuple = tuple(new)
                if new_tuple in self.vertex_fail:
                    continue
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



    def LazySP_Astar(self, weight=1):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        self.vertex_fail = set()
        edge_count, self.vertex_count, search_count = 0, 0, 0
        free_flag = False
        while True:
            if not free_flag:
                search_count += 1
                path_candidate, cost = self.LazySP_Astar_search(start, goal, start_tuple, goal_tuple, weight)
                if path_candidate is None:
                    return None, None
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag:
                print('LazySP_Astar edge_count', edge_count, 'LazySP_Astar search_count', search_count)
                return path_candidate, cost
                # return edge_count, self.vertex_count, cost
            edge_count += 1
            s, t = np.array(e_select[0]), np.array(e_select[1])
            free_flag = self.stspace._state_fp(t)
            if not free_flag:
                self.vertex_fail.add(e_select[1])
            else:
                free_flag = self.stspace._edge_fp(s, t)
            if not free_flag:
                self.collision_edges.add(e_select)
                # array = np.array(self.neighbor_dynamic[e_select[0]])
                # idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                # del self.neighbor_dynamic[e_select[0]][idx]
            self.edges_evaled.add(e_select)

    def LazySP_Astar_search(self, start, goal, start_tuple, goal_tuple, weight):
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
            self.vertex_count += 1
            for new in self.neighbor[current_tuple]:
                new_tuple = tuple(new)
                if new_tuple in self.vertex_fail:
                    continue
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
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.vertex_fail = set()
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}
        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex

        edge_count, vertex_count = 0, 0
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            if current_tuple == goal_tuple:
                print('Astar edge_count', edge_count, 'Astar vertex_count', vertex_count)
                # return edge_count, vertex_count, g_cost[goal_tuple]
                return self.reconstruct_path(came_from, goal), g_cost[goal_tuple]
            if current_tuple in g_cost and current_g > g_cost[current_tuple]:  # current_tuple is already expanded with a lower cost-to-come
                continue  # queue can have repeated vertex with different cost
            vertex_count += 1
            for new in self.neighbor[current_tuple]:
                new_tuple = tuple(new)
                if new_tuple in self.vertex_fail:
                    continue
                edge_count += 1
                if not self.stspace._state_fp(new):
                    self.vertex_fail.add(new_tuple)
                    continue
                if self.stspace._edge_fp(current, new):
                    new_g = current_g + self.E_dist(current, new)
                    if new_tuple not in g_cost or new_g < g_cost[new_tuple]:
                        g_cost[new_tuple] = new_g
                        came_from[new_tuple] = current_tuple
                        heappush(queue, (new_g + weight * self.E_dist(new, goal), new_g, new))
        return None, None

    def edge_Astar(self, weight=1):
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.vertex_fail = set()
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}
        edge_queue = []
        for neb in self.neighbor[start_tuple]:
            f_edge_cost = 0 + self.E_dist(start, neb) + weight * self.E_dist(neb, goal)  # heuristic edge_cost
            heappush(edge_queue, (f_edge_cost, 0, start, neb))  # f_edge_cost, g_cost, source vertex, target vertex

        edge_count, vertex_count = 0, 1
        while edge_queue:
            f_pop, g_pop, source, target = heappop(edge_queue)
            if goal_tuple in g_cost and f_pop >= g_cost[goal_tuple]:
                print('eAstar edge_count', edge_count)
                return self.reconstruct_path(came_from, goal), g_cost[goal_tuple]
                # return edge_count, vertex_count, g_cost[goal_tuple]
            source_tuple = tuple(source)
            target_tuple = tuple(target)
            if target_tuple in self.vertex_fail:
                continue
            edge_dist = self.E_dist(source, target)  # heuristic edge_cost
            target_g = g_pop + edge_dist
            if target_tuple in g_cost and g_cost[target_tuple] <= target_g:
                continue
            edge_count += 1
            if not self.stspace._state_fp(target):
                self.vertex_fail.add(target_tuple)
                continue
            if self.stspace._edge_fp(source, target):
                if target_tuple not in g_cost or target_g < g_cost[
                    target_tuple]:  # true edge_cost after collision checking
                    g_cost[target_tuple] = target_g
                    came_from[target_tuple] = source_tuple
                    if target_tuple == goal_tuple:
                        continue
                    vertex_count += 1
                    for neb in self.neighbor[target_tuple]:
                        neb_g_condidate = target_g + self.E_dist(target, neb)  # heuristic edge_cost
                        if tuple(neb) in g_cost and neb_g_condidate >= g_cost[tuple(neb)]:  # edge is impossible to provide a better sol
                            continue  # thus not added to queue
                        f_edge_cost = neb_g_condidate + weight * self.E_dist(neb, goal)
                        heappush(edge_queue, (f_edge_cost, target_g, target, neb))
        return None, None

    def LWAstar(self, weight=1):
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.vertex_fail = set()
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}
        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex
        edge_queue = []

        edge_count, vertex_count = 0, 0
        while queue or edge_queue:
            if queue:
                minf_v, _, _ = queue[0]  # access the smallest item without popping
            else:
                minf_v = INF  # empty set
            if edge_queue:
                minf_e, _, _, _ = edge_queue[0]
            else:
                minf_e = INF
            if goal_tuple in g_cost and g_cost[goal_tuple] <= min(minf_v, minf_e):
                # print('LWAstar edge_count', edge_count)
                # print(edge_count)
                return edge_count, vertex_count, g_cost[goal_tuple]
            if minf_v <= minf_e:
                _, current_g, current = heappop(queue)
                current_tuple = tuple(current)
                vertex_count += 1
                for neb in self.neighbor[current_tuple]:
                    f_edge_cost = current_g + self.E_dist(current, neb) + weight * self.E_dist(neb, goal)  # heuristic edge_cost
                    heappush(edge_queue, (f_edge_cost, current_g, current, neb))
            else:
                f_pop, g_pop, source, target = heappop(edge_queue)
                source_tuple = tuple(source)
                target_tuple = tuple(target)
                if target_tuple in self.vertex_fail:
                    continue
                edge_dist = self.E_dist(source, target)  # heuristic edge_cost
                target_g = g_pop + edge_dist
                if target_tuple in g_cost and g_cost[target_tuple] <= target_g:
                    continue
                edge_count += 1
                if not self.stspace._state_fp(target):
                    self.vertex_fail.add(target_tuple)
                    continue
                if self.stspace._edge_fp(source, target):
                    if target_tuple not in g_cost or target_g < g_cost[target_tuple]:  # true edge_cost after collision checking
                        g_cost[target_tuple] = target_g
                        came_from[target_tuple] = source_tuple
                        heappush(queue, (target_g + weight * self.E_dist(target, goal), target_g, target))
        return None, None, None

    def reconstruct_path(self, came_from, current):
        current = tuple(current)
        start = tuple(self.init_state)
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        self.path = path
        return path

    def path_sim(self):
        self.stspace.set_config(self.path[0])

        for i in range(len(self.path)-1):
            current = np.array(self.path[i])
            next = np.array(self.path[i+1])
            N = int(self.MH_dist(current, next)/0.05)+1
            for i in range(N):
                joints = current + (i+1) * (next - current) / N
                joint_angles = list(joints)
                self.stspace.joint_angles_control(joint_angles)
                p.stepSimulation()
                time.sleep(1. / 240.)
        print(np.array(self.stspace.get_joint_angles()) - next)

if __name__ == "__main__":
    stspace = KukaEnv(GUI=False)
    env = GymEnvironment(stspace)

    env.reset(6, new_obstacle=True)
    env.sampling(10000)
    env.find_neighbor()
    env.getStartGoal()

    weight = 1
    t0 = time.time()
    path, cost = env.Astar(weight)
    print('Astar time: ', time.time() - t0, 'Astar cost', cost)
    t0 = time.time()
    path, cost = env.edge_Astar(weight)
    print('eAstar time: ', time.time() - t0, 'eAstar cost', cost)
    t0 = time.time()
    path, cost = env.LazySP_Astar(weight)
    print('LazySP time: ', time.time() - t0, 'LazySP cost', cost)

    weight = 2
    t0 = time.time()
    path, cost = env.Astar(weight)
    print('Astar time: ', time.time() - t0, 'Astar cost', cost)
    t0 = time.time()
    path, cost = env.edge_Astar(weight)
    print('eAstar time: ', time.time() - t0, 'eAstar cost', cost)
    t0 = time.time()
    path, cost = env.LazySP_Astar(weight)
    print('LazySP time: ', time.time() - t0, 'LazySP cost', cost)

    obstacles = env.obstacles
    # env.path_sim()
    env.stspace.disconnect()

    sim_flag = True
    if sim_flag:
        stspace_sim = KukaEnv(GUI=True)
        env_sim = GymEnvironment(stspace_sim)
        env_sim.obstacles, env_sim.path = obstacles, path
        env_sim.obs_visual()
        env_sim.path_sim()

    '''
    # print('distance print', np.sum(env.stspace.bound[:env.dim]- env.stspace.bound[env.dim:]))
    sol = path
    print('\nsolution len:',len(sol))
    # print(sol[0].shape)
    if sol:
        gifs = env.stspace.plot(sol, make_gif=True)
        for gif in gifs:
            plt.imshow(gif)
            plt.show()
    sleep(1)
    '''

