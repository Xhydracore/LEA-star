import pickle
from time import time
import numpy as np

'''
x = 1
y = 2
z = 3
output = open('Plans.pkl', 'ab')
pickle.dump(x, output)
pickle.dump(y, output)
pickle.dump(z, output)
output.close()
'''

'''
pkl_file = open('Plans.pkl', 'rb')
print(pickle.load(pkl_file))
print(pickle.load(pkl_file))
print(pickle.load(pkl_file))
pkl_file.close()
'''


def find_neighbor(samples, samples_tuple):
    N = len(samples)
    neighbor = {}
    for i in range(N):
        near = []
        idx_flag = np.sum(abs(samples - samples[i]), axis=1) < 5
        for j in range(N):
            if idx_flag[j] and j!=i:
                near.append(samples[j])
        neighbor[samples_tuple[i]] = near


def find_neighbor_idx(samples):
    N = len(samples)
    neighbor_idx = {}
    for i in range(N):
        near = []
        idx_flag = np.sum(abs(samples - samples[i]), axis=1) < 5
        for j in range(N):
            if idx_flag[j] and j!=i:
                near.append(j)
        neighbor_idx[i] = near

if __name__ == "__main__":
    samples_file = open('samples.pkl', 'rb')
    samples = pickle.load(samples_file)
    samples_tuple = pickle.load(samples_file)
    samples_file.close()

    t0 = time()
    find_neighbor_idx(samples)
    print('time: ', time() - t0)
    t0 = time()
    find_neighbor(samples, samples_tuple)
    print('time: ', time() - t0)

