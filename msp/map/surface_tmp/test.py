import open3d as o3d
import numpy as np

mesh = o3d.io.read_triangle_mesh("indoor.obj")
mesh.compute_triangle_normals()

vertices = np.asarray(mesh.vertices)
triangles = np.asarray(mesh.triangles)
normals = np.asarray(mesh.triangle_normals)

# 例如目标墙面法向为 -Y
target_normal = np.array([0, -1, 0])

# 每个三角形中心
centers = vertices[triangles].mean(axis=1)

# 法向与目标方向接近
cos_angle = normals @ target_normal

mask = cos_angle > np.cos(np.deg2rad(5))

selected_triangles = triangles[mask]

surface_mesh = o3d.geometry.TriangleMesh(
    vertices=o3d.utility.Vector3dVector(vertices),
    triangles=o3d.utility.Vector3iVector(selected_triangles)
)

surface_mesh.remove_unreferenced_vertices()

o3d.io.write_triangle_mesh(
    "indoor_surface.obj",
    surface_mesh
)