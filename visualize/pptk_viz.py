import pptk
import open3d
import numpy as np

# Reference: https://github.com/LightsCameraVision/Point-Cloud-Visualization/blob/main/pc_visualization.ipynb 

def read_file(file):
    pc_bin = np.fromfile(file, '<f4')
    pc_bin = np.reshape(pc_bin, (-1, 4))
    print(pc_bin.shape)
    return pc_bin

def uniform_subsample(points, freq):
    return points[::freq]

def view(points):
    v = pptk.viewer(points)

def main():
    file = '../data/point_clouds/training/velodyne/000111.bin'
    points = read_file(file)
    print(points.shape)
    points = uniform_subsample(points, 3)
    print(points.shape)
    view(points[:, :3])

if __name__ == '__main__':
    main()