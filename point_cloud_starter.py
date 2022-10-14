"""
@article{Zhou2018,
	author    = {Qian-Yi Zhou and Jaesik Park and Vladlen Koltun},
	title     = {{Open3D}: {A} Modern Library for {3D} Data Processing},
	journal   = {arXiv:1801.09847},
	year      = {2018},
}
"""
## IMPORT LIBRARIES
import numpy as np
import time
import open3d as o3d
import pandas as pd
import matplotlib.pyplot as plt

## USE http://www.open3d.org/docs/release/tutorial/Basic/

## CHALLENGE 1 - OPEN A FILE OF YOUR CHOICE AND VISUALIZE THE POINT CLOUD
# The supported extension names are: pcd, ply, xyz, xyzrgb, xyzn, pts.
# RD: Using the basic pcd files from existing kitti

pcd = o3d.io.read_point_cloud("/workspace/src/point_clouds_open3d/my_velodyne.pcd")
print(pcd)
# o3d.visualization.draw_geometries([pcd])

## IF YOU HAVE PPTK INSTALLED, VISUALIZE USING PPTK
# import pptk
# v = pptk.viewer(pcd.points)

## CHALLENGE 2 - VOXEL GRID DOWNSAMPLING
print(f"Points before downsampling: {len(pcd.points)} ")
# TODO(RD): Will have to figure out what is the voxel size unit - should probably in metres.
# Voxel size has to be carefully picked up as the distance between each point cloud point from the LiDAR is also important
# Some possibility that we might miss small obstacles at distance to the LiDAR,
pcd = pcd.voxel_down_sample(voxel_size=0.05)
print(f"Points after downsampling: {len(pcd.points)}")  # DOWNSAMPLING
print(np.asarray(pcd.points))
# o3d.visualization.draw_geometries([pcd])

# ## CHALLENGE 3 - SEGMENTATION
# #TODO(RD): For 1000 iterationsm it took some time like around atleast a second, so probably need to do time analysis here or may be use
# GPU version with the Nvidia GPU
_, inliers = pcd.segment_plane(distance_threshold=0.1, ransac_n=3, num_iterations=1000)
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([1, 0, 0])
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

## CHALLENGE 4 - CLUSTERING USING DBSCAN
# 0.35 and 20 has some good
# labels is an array of the classes assigned to all the
# The max gives the total number of labels/clusters assigned including the "-1" for outliers
# So total clusters = labels.max() + 1
# and each cluster is assigned a color based on the cluster label for each point
# and for the points where label =-1 ,i.e the outliers, the color is black
# the colors array is then reshaped to 3D vector and converted back to the PCD
labels = np.array(pcd.cluster_dbscan(eps=0.35, min_points=30, print_progress=True))
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label) if max_label > 0 else 1)
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd])

# ## BONUS CHALLENGE - CLUSTERING USING KDTREE AND KNN INSTEAD
# pcd_tree =

# ## CHALLENGE 5 - BOUNDING BOXES IN 3D
# bounding_boxes =

# ## CHALLENGE 6 - VISUALIZE THE FINAL RESULTS
# list_of_visuals =

# ## BONUS CHALLENGE 2 - MAKE IT WORK ON A VIDEO
