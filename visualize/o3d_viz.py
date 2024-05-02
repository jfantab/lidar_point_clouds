import os
from glob import glob

import open3d as o3d
import numpy as np
from sklearn import linear_model

## Constants

INDEX = 11

DATE = "2011_10_03"
ID = "0047"
DATA_PATH = f"{DATE}/{DATE}_drive_{ID}_sync"
CAMERA_CALIB_FILE = f"{DATE}/calib_cam_to_cam.txt"

# get RGB camera data
left_image_paths = sorted(glob(os.path.join(DATA_PATH, 'image_02/data/*.png')))
right_image_paths = sorted(glob(os.path.join(DATA_PATH, 'image_03/data/*.png')))

# get LiDAR data
bin_paths = sorted(glob(os.path.join(DATA_PATH, 'velodyne_points/data/*.bin')))

WIDTH = 1242
HEIGHT = 375

## Reading in file
## Conversion to PCD code: https://stackoverflow.com/a/61771198/11826878

def read_bin():
    file = bin_paths[INDEX]
    pc_bin = np.fromfile(file, '<f4') # Reading in the coordinates as floats
    pc_bin = pc_bin.reshape((-1, 4))
    pc_bin = pc_bin[:, :3] # Only taking the X, Y, and Z coordinates
    o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc_bin)) # Converting to Open3D point cloud format
    return o3d_pcd

def read_calib():
    with open(CAMERA_CALIB_FILE,'r') as f:
        calib = f.readlines()

    # get projection matrices (rectified left camera --> left camera (u,v,z))
    P_rect2_cam2 = np.array([float(x) for x in calib[25].strip().split(' ')[1:]]).reshape((3,4))

    # get rigid transformation from Camera 0 (ref) to Camera 2
    R_2 = np.array([float(x) for x in calib[21].strip().split(' ')[1:]]).reshape((3,3))
    t_2 = np.array([float(x) for x in calib[22].strip().split(' ')[1:]]).reshape((3,1))

    # get cam0 to cam2 rigid body transformation in homogeneous coordinates
    T_ref0_ref2 = np.insert(np.hstack((R_2, t_2)), 3, values=[0,0,0,1], axis=0)   

    return (P_rect2_cam2[:,:3], T_ref0_ref2)

def read_image():
    return left_image_paths[INDEX]
    
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

    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    vis.add_geometry(points)

    view_ctl = vis.get_view_control()

    view_ctl.set_front([0, -1, -0.5])  # Adjust these values
    view_ctl.set_lookat([0, 0, 0])  # Focus on the center of the point cloud
    view_ctl.set_up([0, 1, 0])  # Set up vector to Y-axis
    view_ctl.set_zoom(0.5)  # Zoom can be adjusted for better framing

    intrinsic, extrinsic = read_calib()

    # Set the camera parameters
    camera_params = view_ctl.convert_to_pinhole_camera_parameters()
    camera_params.extrinsic = extrinsic

    fx, fy, cx, cy = intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2]
    camera_params.intrinsic.set_intrinsics(WIDTH, HEIGHT, fx, fy, cx, cy)  

    view_ctl.convert_from_pinhole_camera_parameters(camera_params)

    print("{}\n{}".format(camera_params.extrinsic, camera_params.intrinsic.intrinsic_matrix))

    # Run the visualizer
    vis.run()
    vis.destroy_window()

    # image = read_image()

    # fx, fy, cx, cy = intrinsic[0][0], intrinsic[1][1], intrinsic[0][2], intrinsic[1][2]
    
    # intrinsic = o3d.camera.PinholeCameraIntrinsic(WIDTH, HEIGHT, fx,fy, cx, cy)
    # intrinsic.intrinsic_matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
    # cam = o3d.camera.PinholeCameraParameters()
    # cam.intrinsic = intrinsic
    # cam.extrinsic = extrinsic
    # # pcd = o3d.geometry.create_point_cloud_from_rgbd_image(
    # #     image, cam.intrinsic, cam.extrinsic)

    # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(image)
    # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    
    # o3d.visualization.draw_geometries([pcd])


def view_inlier_outlier(points, ind):
    inlier_pcd = points.select_by_index(ind)
    outlier_pcd = points.select_by_index(ind, invert=True)

    outlier_pcd.paint_uniform_color([1, 0, 0])
    inlier_pcd.paint_uniform_color([0.8, 0.8, 0.8])

    o3d.visualization.draw_geometries([inlier_pcd, outlier_pcd])

## Main

def main():
    
    points = read_bin()

    # points = uniform_subsample(points, 5)
    # view(points)

    # _, ind = remove_statistical_outliers(points)
    # _, ind = remove_radius_outliers(points)

    # view_inlier_outlier(points, ind)
    view(points)

if __name__ == '__main__':
    main()