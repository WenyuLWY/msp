import open3d as o3d
import numpy as np

pcd1 = o3d.io.read_point_cloud("map_voxel_tare_room1_wall1.pcd")
pcd2 = o3d.io.read_point_cloud("map_voxel_tare_room1_wall2.pcd")


pcd1.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.2,
        max_nn=10
    )
)

pcd2.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.2,
        max_nn=10
    )
)


# pcd1, ind = pcd1.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
# pcd1 = pcd1.voxel_down_sample(voxel_size=0.05)

geometries = [pcd1,pcd2]
oboxes1 = pcd1.detect_planar_patches(
    normal_variance_threshold_deg=60,
    coplanarity_deg=75,
    outlier_ratio=0.75,
    min_plane_edge_length=0.1,
    min_num_points=4,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=15))
for obox1 in oboxes1:
    # print(obox)
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox1, scale=[1, 1, 1])
    mesh.paint_uniform_color(obox1.color)
    # geometries.append(mesh)
    geometries.append(obox1)

oboxes2 = pcd2.detect_planar_patches(
    normal_variance_threshold_deg=60,
    coplanarity_deg=75,
    outlier_ratio=0.75,
    min_plane_edge_length=0.1,
    min_num_points=4,
    search_param=o3d.geometry.KDTreeSearchParamKNN(knn=15))
for obox2 in oboxes2:
    # print(obox)
    mesh = o3d.geometry.TriangleMesh.create_from_oriented_bounding_box(obox2, scale=[1, 1, 1])
    mesh.paint_uniform_color(obox2.color)
    # geometries.append(mesh)
    geometries.append(obox2)

o3d.visualization.draw_geometries(geometries)
