import pptk
import open3d
import numpy as np

def read_file(file):
    pc_bin = np.fromfile(file, '<f4')
    pc_bin = np.reshape(pc_bin, (-1, 4))
    print(pc_bin.shape)
    return pc_bin

def preprocess(points):
    pass

def uniform_subsample(points, freq):
    return points[::freq]

def view(points):
    v = pptk.viewer(points)

file = 'point_clouds/training/velodyne/000111.bin'
points = read_file(file)
print(points.shape)
points = uniform_subsample(points, 3)
print(points.shape)
view(points[:, :3])
