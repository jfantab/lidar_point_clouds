import pptk
import open3d
import numpy as np

file = 'point_clouds/training/velodyne/000025.bin'
pc_bin = np.fromfile(file, '<f4')
pc_bin = np.reshape(pc_bin, (-1, 4))
print(pc_bin.shape)

v = pptk.viewer(pc_bin[:,:3])
# v.set(point_size=0.5)
