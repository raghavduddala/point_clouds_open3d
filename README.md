# Welcome to the Point Clouds Fast Course

In this course, you will learn to build a 3D Object detection system.
The course can be found here: https://www.thinkautonomous.ai/point-clouds


## DATASET
The dataset used is the KITTI dataset.
I use a subset of 20 point cloud files for this course.
[Link to the full dataset](http://www.cvlibs.net/download.php?file=data_object_velodyne.zip).

The course is currently being completed by using the dockerfile from this repository here: [My Open3D Dockerfile](https://github.com/raghavduddala/3d-obstacle-detection)

## RESULTS
### Downsampled KITTI PCD 
![downsampled_pcd](https://user-images.githubusercontent.com/12818429/195917807-44c000c8-c98b-4568-ab5f-a06a15528374.png)
### Segmentation of the Ground Plane
![Ground_plane_segmentation_kitti_dist_0 1](https://user-images.githubusercontent.com/12818429/195545980-a079ef8c-0027-4780-af7f-a60c21ef300d.png)
### Clustering of detected objects 
![dbscan_clustering](https://user-images.githubusercontent.com/12818429/195917939-e9f87534-13c4-47f7-9ecd-0a23b5269621.png)