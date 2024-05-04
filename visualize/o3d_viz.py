import os
from glob import glob

import open3d as o3d
import numpy as np
from sklearn import linear_model

## Reading in file
## Conversion to PCD code: https://stackoverflow.com/a/61771198/11826878

def read_bin(file):
    # file = bin_paths[INDEX]
    pc_bin = np.fromfile(file, '<f4') # Reading in the coordinates as floats
    pc_bin = pc_bin.reshape((-1, 4))
    pc_bin = pc_bin[:, :3] # Only taking the X, Y, and Z coordinates
    o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_bin)) # Converting to Open3D point cloud format
    return o3d_pcd
    
## Preprocessing
## Reference: https://www.open3d.org/docs/release/tutorial/geometry/pointcloud_outlier_removal.html

def remove_statistical_outliers(points):
    return points.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

def remove_radius_outliers(points):
    return points.remove_radius_outlier(nb_points=4, radius=0.005)

def uniform_subsample(points, freq):
    return points.uniform_down_sample(every_k_points=freq)

## Visualization
## Reference: https://github.com/LightsCameraVision/Point-Cloud-Visualization/blob/main/pc_visualization.ipynb 

def view(points):
    points.paint_uniform_color([0, 0, 0])
    o3d.visualization.draw_geometries([points])


def view_inlier_outlier(points, ind):
    inlier_pcd = points.select_by_index(ind)
    outlier_pcd = points.select_by_index(ind, invert=True)

    outlier_pcd.paint_uniform_color([1, 0, 0])
    inlier_pcd.paint_uniform_color([0.8, 0.8, 0.8])

    o3d.visualization.draw_geometries([inlier_pcd, outlier_pcd])

## Main

def main():
    points = read_bin("../data/point_clouds/training/velodyne/000008.bin")

    # points = uniform_subsample(points, 5)
    # view(points)

    # _, ind = remove_statistical_outliers(points)
    # _, ind = remove_radius_outliers(points)

    # view_inlier_outlier(points, ind)
    view(points)

if __name__ == '__main__':
    main()