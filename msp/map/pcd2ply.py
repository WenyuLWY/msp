import open3d as o3d
pcd = o3d.io.read_point_cloud("./indoor152/map_voxel2.pcd")
o3d.io.write_point_cloud("./indoor152/map_imad_voxel.ply", pcd)   # 或 .ply / .xyz
