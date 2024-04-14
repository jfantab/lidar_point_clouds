import open3d as o3d
import numpy as np

def read_file(file):
    pc_bin = np.fromfile(file, '<f4')
    pc_bin = pc_bin.reshape((-1, 4))
    pc_bin = pc_bin[:, :3] 
    o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_bin))
    o3d_pcd.paint_uniform_color([0, 0, 0])
    return o3d_pcd

def remove_outliers(points):
    pass
    # return points.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

def uniform_subsample(points, freq):
    return points.uniform_down_sample(every_k_points=freq)

def view(points):
    o3d.visualization.draw_geometries([points])

file = 'point_clouds/training/velodyne/000111.bin'
points = read_file(file)
points = uniform_subsample(points, 5)
# points = remove_outliers(points)
view(points)
