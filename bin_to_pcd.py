import open3d as o3d
import numpy as np
import struct

size_float = 4
list_pcd = []
# TODO(RD): File paths have to be changed to automated using the os or sys. paths
# Path for the raw file was from downloads, the file has to be copied or moved to path of docker image launch folder
file_to_open = ""
file_to_save = "/workspace/code/ta/src/point_clouds_open3d/my_velodyne.pcd"
with open(file_to_open, "rb") as f:
    byte = f.read(size_float * 4)
    while byte:
        x, y, z, intensity = struct.unpack("ffff", byte)
        list_pcd.append([x, y, z])
        byte = f.read(size_float * 4)
np_pcd = np.asarray(list_pcd)
pcd = o3d.geometry.PointCloud()
v3d = o3d.utility.Vector3dVector
pcd.points = v3d(np_pcd)

o3d.io.write_point_cloud(file_to_save, pcd)
