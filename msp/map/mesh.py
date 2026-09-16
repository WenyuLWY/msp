import trimesh
import open3d as o3d

if __name__ == "__main__":
    GT_FILE = "indoor.dae"
    scene_or_mesh = trimesh.load(GT_FILE, force='mesh')
    # 如果 dae 里有多个子网格,trimesh.load 默认返回 Scene,需要合并
    if isinstance(scene_or_mesh, trimesh.Scene):
        gt_mesh = trimesh.util.concatenate(
            [g for g in scene_or_mesh.geometry.values()]
        )
    else:
        gt_mesh = scene_or_mesh

    gt_mesh.export('indoor.obj')  # 或 .ply / .stl

    # o3d_mesh = o3d.io.read_triangle_mesh('indoor.obj')
    # o3d_mesh.compute_vertex_normals()
    print("Mesh loaded and normals computed.")