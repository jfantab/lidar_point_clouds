# lidar_point_clouds

## Introduction

Each of the folders here correspond to the various tasks of our project.

The `visualize` folder holds the Python files for visualizing in Open3D and PPTK.

The `fuse` folder holds the code for performing sensor fusion as well as the MATLAB developer kit code from the KITTI website. The original code can be found [here](https://github.com/itberrios/CV_tracking/blob/main/kitti_tracker/1_kitti_object_detection_lidar.ipynb).

The `segmentation` folder holds the code for filtering via segmentation.

The `clustering` folder holds the code for performing various clustering methods on the point clouds.

The `bboxes` folder holds the code for filtering via bounding boxes.

The `misc` folder holds various Bash scripts for modifying the data formats or extracing a sample of the data.

The `data` folder is ignored in the `.gitignore`. The `data_loader.ipynb` file has the commands to download the data from the KITTI website.

## Instructions

Create a Conda virtual environment with `conda create -n lidar python=3.6`. Open3D and PPTK only work on Python 3.6.

Run `mkdir data` to create the data folder.

