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
# points_3d = np.asarray(pcd.points)
# print(points_3d.shape)
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
print(labels)
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")
colors = plt.get_cmap("tab20")(labels / (max_label) if max_label > 0 else 1)
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([pcd])

# ## BONUS CHALLENGE - CLUSTERING USING KDTREE AND KNN INSTEAD
# pcd_tree =

# ## CHALLENGE 5 - BOUNDING BOXES IN 3D
points_3d = np.asarray(pcd.points)
# Size of pcd.points = (84500,3)
# Size of labels = (84500,)
# Making use of the pandas dataframe as it comes with open3d to find out the indices of the object
# We need to find the indices of point clouds with same labels grouped togther
# groupby.apply() gives the labels and their list of indices as a dictionary key and pair
# indices will be a list of list with the indices of the point clouds w.r.t labels
# appending the bounding box in the loop and not the point cloud points as they will be already present
# Setting a threshold will be good for the number of point clouds to avoid unnecessary bounding boxes
indices = pd.Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()
bboxes = []
POINTS_MIN_THRESHOLD = 30
POINTS_MAX_THRESHOLD = 650

for i in range(len(indices)):
    object_pcd = pcd.select_by_index(indices[i])
    points_object_pcd = len(np.array(object_pcd.points))
    if (
        points_object_pcd > POINTS_MIN_THRESHOLD
        and points_object_pcd < POINTS_MAX_THRESHOLD
    ):
        bbox = object_pcd.get_axis_aligned_bounding_box()
        bbox.color = (0, 0, 0)
        bboxes.append(bbox)

pcd_bbox = []
pcd_bbox.append(pcd)
pcd_bbox.extend(bboxes)
o3d.visualization.draw_geometries(pcd_bbox)

# labels = np.reshape(())

# for i in range(len()
# list_pcd = np.asarray(pcd.)
# print(list_pcd)
# bounding_boxes =

# ## CHALLENGE 6 - VISUALIZE THE FINAL RESULTS
# list_of_visuals =

# ## BONUS CHALLENGE 2 - MAKE IT WORK ON A VIDEO

