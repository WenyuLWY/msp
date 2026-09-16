import open3d as o3d
import numpy as np

# 读取点云
# pcd = o3d.io.read_point_cloud("map_voxel_tare_room1_wall1.pcd")
pcd = o3d.io.read_point_cloud("map_imad1_room1.pcd")


# RANSAC 平面分割
plane_model, inliers = pcd.segment_plane(
    distance_threshold=0.05,  # 点到平面的最大距离，3 cm
    ransac_n=3,               # 每次随机选择 3 个点拟合平面
    num_iterations=1000       # RANSAC 迭代次数
)

# 平面方程 ax + by + cz + d = 0
a, b, c, d = plane_model

print(f"Plane: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")
print("Plane point number:", len(inliers))





# 提取平面点
plane_cloud = pcd.select_by_index(inliers)
# 提取剩余点
remaining_cloud = pcd.select_by_index(inliers, invert=True)
# 可视化
plane_cloud.paint_uniform_color([1.0, 0.0, 0.0])
remaining_cloud.paint_uniform_color([0.7, 0.7, 0.7])

# 1.
normal = np.array([a, b, c])
normal = normal / np.linalg.norm(normal)
print(f"Plane normal: [{normal[0]:.2f}, {normal[1]:.2f}, {normal[2]:.2f}]")

# 2.
z_axis = np.array([0.0, 0.0, 1.0])
angle_to_z = np.degrees(
    np.arccos(
        np.clip(np.abs(np.dot(normal, z_axis)), -1.0, 1.0)
    )
)
verticality_angle = abs(90.0 - angle_to_z)
print(f"Verticality deviation: {verticality_angle:.3f} deg")

# 3.
points = np.asarray(plane_cloud.points)
distances = np.abs(
    a * points[:, 0] +
    b * points[:, 1] +
    c * points[:, 2] +
    d
) / np.sqrt(a*a + b*b + c*c)
print(f"Mean surface deviation: {np.mean(distances)*1000:.2f} mm")
print(f"RMSE surface deviation: {np.sqrt(np.mean(distances**2))*1000:.2f} mm")
print(f"Max surface deviation: {np.max(distances)*1000:.2f} mm")

# 4.
points = np.asarray(plane_cloud.points)
min_xyz = np.min(points, axis=0)
max_xyz = np.max(points, axis=0)
print(f"Min: [{min_xyz[0]:.2f}, {min_xyz[1]:.2f}, {min_xyz[2]:.2f}]")
print(f"Max: [{max_xyz[0]:.2f}, {max_xyz[1]:.2f}, {max_xyz[2]:.2f}]")
print(f"X range: {max_xyz[0]-min_xyz[0]:.3f} m")
print(f"Y range: {max_xyz[1]-min_xyz[1]:.3f} m")
print(f"Z range: {max_xyz[2]-min_xyz[2]:.3f} m")


# 5.
points = np.asarray(plane_cloud.points)

# 平面法向量
normal = np.array([a, b, c], dtype=float)
normal /= np.linalg.norm(normal)

# 平面上的一个点
plane_center = -d * np.array([a, b, c]) / (a*a + b*b + c*c)

# 在平面上构造两个正交方向 u, v
ref = np.array([0.0, 0.0, 1.0])

# 避免 ref 和 normal 平行
if abs(np.dot(normal, ref)) > 0.9:
    ref = np.array([1.0, 0.0, 0.0])

u = np.cross(normal, ref)
u /= np.linalg.norm(u)

v = np.cross(normal, u)
v /= np.linalg.norm(v)

# 把墙面点投影到局部二维坐标系
relative = points - plane_center

u_coord = relative @ u
v_coord = relative @ v

u_min, u_max = u_coord.min(), u_coord.max()
v_min, v_max = v_coord.min(), v_coord.max()

# 四个角点
vertices = np.array([
    plane_center + u_min*u + v_min*v,
    plane_center + u_max*u + v_min*v,
    plane_center + u_max*u + v_max*v,
    plane_center + u_min*u + v_max*v
])

triangles = np.array([
    [0, 1, 2],
    [0, 2, 3]
])

plane_mesh = o3d.geometry.TriangleMesh(
    o3d.utility.Vector3dVector(vertices),
    o3d.utility.Vector3iVector(triangles)
)

plane_mesh.compute_vertex_normals()
plane_mesh.paint_uniform_color([0.0, 0.5, 1.0])


# ============================================================
# 6. 与 Ground Truth OBJ mesh 比较
# ============================================================

# 读取真实墙面 mesh
gt_mesh = o3d.io.read_triangle_mesh("indoor_surface.obj")
gt_mesh.compute_vertex_normals()

print("GT mesh vertices:", len(gt_mesh.vertices))
print("GT mesh triangles:", len(gt_mesh.triangles))


# ------------------------------------------------------------
# 6.1 使用 RaycastingScene 计算 point-to-mesh distance
# ------------------------------------------------------------

# legacy mesh -> tensor mesh
gt_mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(gt_mesh)

scene = o3d.t.geometry.RaycastingScene()
scene.add_triangles(gt_mesh_t)


# 推荐：
# 如果 pcd 已经裁剪得只剩这一面墙，就使用 pcd
# 如果 pcd 中还包含其他墙/地面，则可以暂时使用 plane_cloud
scan_points = np.asarray(plane_cloud.points).astype(np.float32)

query_points = o3d.core.Tensor(
    scan_points,
    dtype=o3d.core.Dtype.Float32
)

distances_gt = scene.compute_distance(query_points).numpy()


print("\n========== Scan-to-GT Mesh Error ==========")

print(f"Mean error: {np.mean(distances_gt)*1000:.2f} mm")

print(
    f"RMSE error: "
    f"{np.sqrt(np.mean(distances_gt**2))*1000:.2f} mm"
)

print(f"Median error: {np.median(distances_gt)*1000:.2f} mm")

print(
    f"P95 error: "
    f"{np.percentile(distances_gt, 95)*1000:.2f} mm"
)

print(f"Max error: {np.max(distances_gt)*1000:.2f} mm")


o3d.visualization.draw_geometries(
    [
        remaining_cloud,
        plane_cloud,
        plane_mesh
    ],
    mesh_show_back_face=True
)