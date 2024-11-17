from kuka_env import KukaEnv
import pybullet as p
import numpy as np
import math
from time import sleep, time
import matplotlib.pyplot as plt
import pickle

from heapq import heappop, heappush
INF = float('inf')

class GymEnvironment():
    def __init__(self, statespace, save_obs_samples = False, encoder_model = None) -> None:

        self.stspace = statespace
        self.save_obs_samples = save_obs_samples

        self.obstacle_idx = []
        self.obstacles = []
        self.neighbor = {}

        self.dim = self.stspace.config_dim 
        self.bound_norm = np.linalg.norm(self.stspace.bound[self.dim:] - self.stspace.bound[:self.dim])
    
    def reset(self, sampleStandGoal = True, new_obstacle=True):
        """Resets the Environment
        """

        ### environment change
        if new_obstacle:
            for idx in self.obstacle_idx:
                self.stspace.remove_body(idx)
            self.obstacle_idx.clear()
            self.populateRandomObstacles(20)
            if self.save_obs_samples:
                output = open('obs.pkl', 'wb')
                pickle.dump(self.obstacles, output)
                output.close()
        
        if sampleStandGoal:
            ### sample new start and goal
            tableTop = True
            if tableTop: 
                # try to sample config within the workspace 
                max_config_z = .6 * np.max([it[1][2] + it[0][2] for it in self.obstacles[:-2]]) 
                min_config_y = .1 + np.min([it[1][1] + it[0][1] for it in self.obstacles[:-2]])
                constraint_fn = lambda x : np.logical_and(x > np.array([-.7, min_config_y, .0125]),
                                                          x < np.array([.7, math.inf, max_config_z])).all()

                self.stspace.set_random_init_goal(dist=self.bound_norm / 20, dist_worksp=0.1, constraint_fn=constraint_fn)
            else:
                self.stspace.set_random_init_goal(dist=self.bound_norm / 10, dist_worksp=0.5)  # may need clearance
    
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
        n = 4
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

    '''
    def calcHeurisitic(self, state, normalized=False):
        dist = self.stspace.distance(state, self.stspace.goal_state)
        if normalized:
            return dist / self.bound_norm
        return dist
    '''

    def obs_visual(self):
        for halfExtents, basePosition in self.obstacles:
            body_idx = self.stspace.create_voxel(halfExtents, basePosition)
            self.obstacle_idx.append(body_idx)

    def dataUnpack(self):
        obs_file = open('obs.pkl', 'rb')
        self.obstacles = pickle.load(obs_file)
        obs_file.close()
        self.obs_visual()

        samples_file = open('samples.pkl', 'rb')
        self.samples = pickle.load(samples_file)
        self.samples_tuple = pickle.load(samples_file)
        samples_file.close()

        neighbor_file = open('neighbor.pkl', 'rb')
        self.neighbor = pickle.load(neighbor_file)
        neighbor_file.close()

    def MH_dist(self, p, q):
        return sum(abs(p - q))

    def sampling(self, n):
        samples = self.stspace.sample_n_points(n)
        samples = np.append([self.stspace.init_state, self.stspace.goal_state], samples, axis=0)
        samples_tuple = []
        for i in range(len(samples)):
            samples_tuple.append(tuple(samples[i]))
        self.samples, self.samples_tuple = samples, samples_tuple
        if self.save_obs_samples:
            output = open('samples.pkl', 'ab')
            pickle.dump(self.samples, output)
            pickle.dump(self.samples_tuple, output)
            output.close()

    def find_neighbor(self):
        samples, samples_tuple = self.samples, self.samples_tuple
        N = len(samples)
        for i in range(N):
            near = []
            idx_flag = np.sum(abs(samples - samples[i]), axis=1) < 5
            for j in range(N):
                if idx_flag[j] and j != i:
                    near.append(samples[j])
            self.neighbor[samples_tuple[i]] = near
        if self.save_obs_samples:
            output = open('neighbor.pkl', 'wb')
            pickle.dump(self.neighbor, output)
            output.close()

    def find_neighbor_idx(self):
        samples = self.samples
        N = len(samples)
        neighbor_idx = []
        for i in range(N):
            near = []
            idx_flag = np.sum(abs(samples - samples[i]), axis=1) < 5
            for j in range(N):
                if idx_flag[j] and j != i:
                    near.append(j)
            neighbor_idx.append(near)
        self.neighbor_idx = neighbor_idx
        if self.save_obs_samples:
            output = open('neighbor_idx.pkl', 'wb')
            pickle.dump(self.neighbor_idx, output)
            output.close()

    def Astar_useidx(self, weight=1):
        samples = self.samples
        start, goal = 0, 1
        goal_state=samples[goal]
        came_from = {}
        g_cost = {}
        came_from[0] = None
        g_cost[0] = 0
        queue = [(weight*self.MH_dist(samples[start], goal_state), 0, start)]  # f_cost, g_cost, vertex

        vertex_count, edge_count = 0, 0
        while queue:
            _, current_g, current = heappop(queue)
            if current == goal:
                print('Astar edge_count', edge_count, 'Astar vertex_count', vertex_count)
                return g_cost[goal]
            if current in g_cost and current_g > g_cost[current]:  # current_tuple is already expanded with a lower cost-to-come
                continue                                                       # queue can have repeated vertex with different cost
            vertex_count += 1
            for new in self.neighbor_idx[current]:
                edge_count += 1
                new_state = samples[new]
                if self.stspace._edge_fp(samples[current], new_state):
                    new_g = current_g + self.MH_dist(samples[current], new_state)
                    if new not in g_cost or new_g < g_cost[new]:
                        g_cost[new] = new_g
                        came_from[new] = current
                        heappush(queue, (new_g + weight*self.MH_dist(new_state, goal_state), new_g, new))
        return None

    def Astar(self, weight=1):
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        came_from = {}
        g_cost = {}
        came_from[start_tuple] = None
        g_cost[start_tuple] = 0
        queue = [(weight*self.MH_dist(start, goal), 0, start)]  # f_cost, g_cost, vertex

        vertex_count, edge_count = 0, 0
        while queue:
            _, current_g, current = heappop(queue)
            current_tuple = tuple(current)
            if current_tuple == goal_tuple:
                print('Astar edge_count', edge_count, 'Astar vertex_count', vertex_count)
                return self.reconstruct_path(came_from), g_cost[goal_tuple]
            if current_tuple in g_cost and current_g > g_cost[current_tuple]:  # current_tuple is already expanded with a lower cost-to-come
                continue                                                       # queue can have repeated vertex with different cost
            vertex_count += 1
            for new in self.neighbor[current_tuple]:
                edge_count += 1
                if self.stspace._edge_fp(current, new):
                    new_g = current_g + self.MH_dist(current, new)
                    new_tuple = tuple(new)
                    if new_tuple not in g_cost or new_g < g_cost[new_tuple]:
                        g_cost[new_tuple] = new_g
                        came_from[new_tuple] = current_tuple
                        heappush(queue, (new_g + weight*self.MH_dist(new, goal), new_g, new))
        return None, None

    def edge_Astar(self, weight=1):
        start, goal = self.samples[0], self.samples[1]
        start_tuple, goal_tuple = tuple(start), tuple(goal)
        came_from = {}
        g_cost = {}
        came_from[start_tuple] = None
        g_cost[start_tuple] = 0
        edge_queue = []
        for neb in self.neighbor[start_tuple]:
            f_edge_cost = 0 + self.MH_dist(start, neb) + weight*self.MH_dist(neb, goal)  # heuristic edge_cost
            heappush(edge_queue, (f_edge_cost, 0, start, neb))  # f_edge_cost, g_cost, source vertex, target vertex

        edge_count = 0
        while edge_queue:
            f_pop, g_pop, source, target = heappop(edge_queue)
            if goal_tuple in g_cost and f_pop >= g_cost[goal_tuple]:
                print('eAstar edge_count', edge_count)
                return self.reconstruct_path(came_from), g_cost[goal_tuple]
            source_tuple = tuple(source)
            # g_pop > g_cost[source_tuple]:   # not need for eA*
            #     continue
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
                            f_edge_cost = neb_g_condidate + weight*self.MH_dist(neb, goal)
                            heappush(edge_queue, (f_edge_cost, target_g, target, neb))
        return None

    def reconstruct_path(self, came_from):
        current =  self.samples_tuple[1]
        start =  self.samples_tuple[0]
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
                sleep(1. / 240.)
        print(np.array(self.stspace.get_joint_angles()) - next)

if __name__ == "__main__":
    stspace = KukaEnv(GUI=False)
    env = GymEnvironment(stspace, save_obs_samples=False)

    # states = env.stspace.sample_n_points(2)
    # env.goal = states[1]
    # end_point = env.stspace.get_robot_points()
    # print('\n\nend point',end_point)

    useSavedData = True
    if useSavedData:
        env.dataUnpack()
    else:
        env.reset(new_obstacle=True)
        env.sampling(2000)
        env.find_neighbor()

    t0 = time()
    path, cost = env.Astar(weight=1)
    print('Astar time: ', time() - t0, 'Astar cost', cost)
    t0 = time()
    path, cost = env.edge_Astar(weight=1)
    print('eAstar time: ', time() - t0, 'eAstar cost', cost)
    obstacles = env.obstacles
    # env.path_sim()
    env.stspace.disconnect()

    sim_flag = False
    if sim_flag:
        stspace_sim = KukaEnv(GUI=True)
        env_sim = GymEnvironment(stspace_sim, save_obs_samples=False)
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

