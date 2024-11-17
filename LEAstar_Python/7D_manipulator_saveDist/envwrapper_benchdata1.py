from kuka_env_bench1 import KukaEnv
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
        self.Distance_goal = {}

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
        '''
        samples_all = self.stspace.sample_n_points(n)
        samples_all = np.round(samples_all, 2)
        samples = []
        for i in range(len(samples_all)):
            if self.stspace._state_fp(samples_all[i]):
                samples.append(samples_all[i])
        '''
        samples = self.stspace.sample_n_points(n)
        samples = np.round(samples, 3)

        samples_tuple = []
        for i in range(len(samples)):
            samples_tuple.append(tuple(samples[i]))
        self.samples, self.samples_tuple = samples, samples_tuple

    def getStartGoal(self):
        dist = 0
        self.init_state, self.goal_state = None, None
        while dist < 1:
            start_idx = random.randint(0, len(self.samples) - 1)
            if self.stspace._state_fp(self.samples[start_idx]):
                self.init_state = self.samples[start_idx]
            goal_idx = random.randint(0, len(self.samples)-1)
            if self.stspace._state_fp(self.samples[goal_idx]):
                self.goal_state = self.samples[goal_idx]
            if self.init_state is not None and self.goal_state is not None:
                dist = self.E_dist(self.init_state, self.goal_state)
        distance_to_goal = np.sqrt(np.sum((abs(self.samples - self.samples[goal_idx])) ** 2, axis=1))
        for i in range(len(self.samples)):
            self.Distance_goal[self.samples_tuple[i]] = distance_to_goal[i]

    def find_neighbor(self):
        self.neighbor = {}
        samples, samples_tuple = self.samples, self.samples_tuple
        N = len(samples)
        gamma = 7
        ner = min(3, gamma * (math.log10(N+1)/N)**(1/7))
        for i in range(N):
            near = []
            Dist = np.sqrt(np.sum((abs(samples - samples[i])) ** 2, axis=1))
            idx_flag = Dist < ner
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
        ts = time.perf_counter()
        while True:
            if not free_flag:
                search_count += 1
                path_candidate, cost = self.LRAstar_search(start, goal, start_tuple, goal_tuple, weight, ts)
                if path_candidate is None:
                    return None, None, None
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag and path_candidate[-1] == goal_tuple:
                # print('LRAstar e_count', edge_count, 'LRAstar search_count', search_count, 'LRAstar v_count', self.vertex_count)
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

    def LRAstar_search(self, start, goal, start_tuple, goal_tuple, weight, ts):
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}

        queue = [(weight * self.Distance_goal[start_tuple], 0, start)]  # f_cost, g_cost, vertex
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            flag, Path = self.Event(current, came_from)
            if time.perf_counter() - ts > 15:
                return None, None
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
                    heappush(queue, (new_g + weight * self.Distance_goal[new_tuple], new_g, new))
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



    def LazySP_Astar(self, weight):
        # self.neighbor_dynamic = copy.deepcopy(self.neighbor)
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.edges_evaled = set()
        self.collision_edges = set()
        self.vertex_fail = set()
        edge_count, self.vertex_count, search_count = 0, 0, 0
        free_flag = False
        ts = time.perf_counter()
        while True:
            if not free_flag:
                search_count += 1
                path_candidate, cost = self.LazySP_Astar_search(start, goal, start_tuple, goal_tuple, weight, ts)
                if path_candidate is None:
                    return None, None, None
            solution_flag, e_select = self.Path_is_evaluated(path_candidate)
            if solution_flag:
                # print('LazySP_Astar e_count', edge_count, 'LazySP_Astar search_count', search_count, 'LazySP_Astar v_count', self.vertex_count)
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
                # array = np.array(self.neighbor_dynamic[e_select[0]])
                # idx = np.where(np.sum(array - np.array(e_select[1]), axis=1) == 0)[0][0]
                # del self.neighbor_dynamic[e_select[0]][idx]
            self.edges_evaled.add(e_select)

    def LazySP_Astar_search(self, start, goal, start_tuple, goal_tuple, weight, ts):
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}

        queue = [(weight * self.Distance_goal[start_tuple], 0, start)]  # f_cost, g_cost, vertex
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            if time.perf_counter() - ts > 15:
                return None, None
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
                    heappush(queue, (new_g + weight * self.Distance_goal[new_tuple], new_g, new))
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


    def Astar(self, weight):
        start, goal = self.init_state, self.goal_state
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        self.vertex_fail = set()
        came_from = {start_tuple: None}
        g_cost = {start_tuple: 0}
        queue = [(weight * self.E_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex
        ts = time.perf_counter()
        edge_count, vertex_count = 0, 0
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            if time.perf_counter() - ts > 10:
                return None, None, None
            if current_tuple == goal_tuple:
                # print('Astar e_count', edge_count, 'Astar v_count', vertex_count)
                return edge_count, vertex_count, g_cost[goal_tuple]
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
        return None, None, None

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
                # print('eAstar e_count', edge_count, 'eAstar v_count', vertex_count)
                return edge_count, vertex_count, g_cost[goal_tuple]
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
                        neb_tuple = tuple(neb)
                        if neb_tuple in g_cost and neb_g_condidate >= g_cost[neb_tuple]:  # edge is impossible to provide a better sol
                            continue  # thus not added to queue
                        f_edge_cost = neb_g_condidate + weight * self.E_dist(neb, goal)
                        heappush(edge_queue, (f_edge_cost, target_g, target, neb))
        return None, None, None

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
                # print('LWAstar e_count', edge_count, 'LWAstar v_count', vertex_count)
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
    GS = GymEnvironment(stspace)

    sample_size = 10000  # [1000, 5000, 10000]
    GS.sampling(sample_size)

    GS.find_neighbor()
    num_of_obs = 10  # [2, 6, 10]
    depth_1, depth_2 = 2, 4
    for i in range(1):
        data = []
        GS.reset(num_of_obs, new_obstacle=True)
        ts = time.time()
        for _ in range(20):
            GS.getStartGoal()

            weight = 1
            t0 = time.perf_counter()
            e_c1, v_c1, cost1 = GS.Astar(weight)
            t_Astar = time.perf_counter() - t0
            if e_c1 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c2, v_c2, cost2 = GS.LWAstar(weight)
            t_LWAstar = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c3, v_c3, cost3 = GS.edge_Astar(weight)
            t_LazyEAstar = time.perf_counter()- t0
            t0 = time.perf_counter()
            e_c4, v_c4, cost4 = GS.LazySP_Astar(weight)
            t_LazySP_Astar = time.perf_counter() - t0
            if e_c4 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c5, v_c5, cost5 = GS.LRAstar(depth_1, weight)
            t_LRAstar = time.perf_counter() - t0
            if e_c5 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c6, v_c6, cost6 = GS.LRAstar(depth_2, weight)
            t_LRAstar_1 = time.perf_counter() - t0
            if e_c6 is None:
                print('timeout')
                continue
            '''
            t0 = time.perf_counter()
            e_c6, v_c6, cost6 = GS.LazySP_LPAstar()
            t_LazySP_LPAstar = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c7, v_c7, cost7 = GS.LRAstar_LPAstar()
            t_LRAstar_LPAstar = time.perf_counter()- t0
            '''

            weight = 1.5
            t0 = time.perf_counter()
            e_c11, v_c11, cost11 = GS.Astar(weight)
            t_Astar1 = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c12, v_c12, cost12 = GS.LWAstar(weight)
            t_LWAstar1 = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c13, v_c13, cost13 = GS.edge_Astar(weight)
            t_LazyEAstar1 = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c14, v_c14, cost14 = GS.LazySP_Astar(weight)
            t_LazySP_Astar1 = time.perf_counter() - t0
            if e_c14 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c15, v_c15, cost15 = GS.LRAstar(depth_1, weight)
            t_LRAstar1 = time.perf_counter() - t0
            if e_c15 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c16, v_c16, cost16 = GS.LRAstar(depth_2, weight)
            t_LRAstar1_1 = time.perf_counter() - t0
            if e_c16 is None:
                print('timeout')
                continue

            weight = 2
            t0 = time.perf_counter()
            e_c21, v_c21, cost21 = GS.Astar(weight)
            t_Astar2 = time.perf_counter()- t0
            t0 = time.perf_counter()
            e_c22, v_c22, cost22 = GS.LWAstar(weight)
            t_LWAstar2 = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c23, v_c23, cost23 = GS.edge_Astar(weight)
            t_LazyEAstar2 = time.perf_counter()- t0
            t0 = time.perf_counter()
            e_c24, v_c24, cost24 = GS.LazySP_Astar(weight)
            t_LazySP_Astar2 = time.perf_counter() - t0
            if e_c24 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c25, v_c25, cost25 = GS.LRAstar(depth_1, weight)
            t_LRAstar2 = time.perf_counter() - t0
            if e_c25 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c26, v_c26, cost26 = GS.LRAstar(depth_2, weight)
            t_LRAstar2_1 = time.perf_counter() - t0
            if e_c26 is None:
                print('timeout')
                continue

            weight = 2.5
            t0 = time.perf_counter()
            e_c31, v_c31, cost31 = GS.Astar(weight)
            t_Astar3 = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c32, v_c32, cost32 = GS.LWAstar(weight)
            t_LWAstar3 = time.perf_counter()- t0
            t0 = time.perf_counter()
            e_c33, v_c33, cost33 = GS.edge_Astar(weight)
            t_LazyEAstar3 = time.perf_counter() - t0
            t0 = time.perf_counter()
            e_c34, v_c34, cost34 = GS.LazySP_Astar(weight)
            t_LazySP_Astar3 = time.perf_counter() - t0
            if e_c34 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c35, v_c35, cost35 = GS.LRAstar(depth_1, weight)
            t_LRAstar3 = time.perf_counter() - t0
            if e_c35 is None:
                print('timeout')
                continue
            t0 = time.perf_counter()
            e_c36, v_c36, cost36 = GS.LRAstar(depth_2, weight)
            t_LRAstar3_1 = time.perf_counter()- t0
            if e_c36 is None:
                print('timeout')
                continue

            data = data + [[t_Astar, e_c1, v_c1, cost1, t_LWAstar, e_c2, v_c2, cost2, t_LazyEAstar, e_c3, v_c3, cost3,
                            t_LazySP_Astar, e_c4, v_c4, cost4, t_LRAstar, e_c5, v_c5, cost5, t_LRAstar_1, e_c6, v_c6, cost6,
                            t_Astar1, e_c11, v_c11, cost11, t_LWAstar1, e_c12, v_c12, cost12, t_LazyEAstar1, e_c13,
                            v_c13, cost13,  t_LazySP_Astar1, e_c14, v_c14, cost14, t_LRAstar1, e_c15, v_c15, cost15,
                            t_LRAstar1_1, e_c16, v_c16, cost16,
                            t_Astar2, e_c21, v_c21, cost21, t_LWAstar2, e_c22, v_c22, cost22, t_LazyEAstar2, e_c23,
                            v_c23, cost23, t_LazySP_Astar2, e_c24, v_c24, cost24, t_LRAstar2, e_c25, v_c25, cost25,
                            t_LRAstar2_1, e_c26, v_c26, cost26,
                            t_Astar3, e_c31, v_c31, cost31, t_LWAstar3, e_c32, v_c32, cost32, t_LazyEAstar3, e_c33,
                            v_c33, cost33, t_LazySP_Astar3, e_c34, v_c34, cost34, t_LRAstar3, e_c35, v_c35, cost35,
                            t_LRAstar3_1, e_c36, v_c36, cost36]]
        print(time.time()-ts)
        scipy.io.savemat('7D_S' + str(sample_size) + '_OBS' + str(num_of_obs) + str(i) + '.mat', mdict={'data': data})
    stspace.disconnect()
