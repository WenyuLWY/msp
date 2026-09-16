import open3d as o3d
import numpy as np


def evaluate_wall_scan(
    scan_pcd_path,
    gt_mesh_path,
    association_threshold=0.10,   # 扫描点距离 GT mesh 小于多少认为属于该墙面
    coverage_threshold=0.05,      # GT 表面到扫描点距离小于多少认为该区域已覆盖
    gt_sample_num=20000,          # GT mesh 表面采样点数
    save_wall_path="extracted_wall.pcd"
):
    """
    评价扫描点云对单个 GT 墙面的扫描质量和平整度。

    Parameters
    ----------
    scan_pcd_path : str
        包含多个墙面的完整扫描点云。
    gt_mesh_path : str
        单个真实墙面的 mesh，例如 indoor_surface.obj。
    association_threshold : float
        扫描点到 GT mesh 的最大距离，单位 m。
        小于该距离的扫描点被认为属于这个墙面。
    coverage_threshold : float
        GT 表面采样点到扫描墙面点云的最大距离，单位 m。
        小于该距离认为 GT 区域已被扫描覆盖。
    gt_sample_num : int
        GT mesh 表面均匀采样的点数。
    save_wall_path : str
        提取出的墙面点云保存路径。

    Returns
    -------
    dict
        所有评价指标。
    """

    # ============================================================
    # 1. 读取扫描点云
    # ============================================================
    print("\n========================================")
    print("Loading scan point cloud...")
    print("========================================")

    pcd = o3d.io.read_point_cloud(scan_pcd_path)

    if not pcd.has_points():
        raise ValueError(f"Cannot read point cloud: {scan_pcd_path}")

    scan_points = np.asarray(pcd.points)

    print(f"Scan point number: {len(scan_points)}")


    # ============================================================
    # 2. 读取 GT mesh
    # ============================================================
    print("\n========================================")
    print("Loading GT mesh...")
    print("========================================")

    gt_mesh = o3d.io.read_triangle_mesh(gt_mesh_path)

    if len(gt_mesh.vertices) == 0:
        raise ValueError(f"Cannot read mesh: {gt_mesh_path}")

    gt_vertices = np.asarray(gt_mesh.vertices)

    print(f"GT mesh vertices: {len(gt_mesh.vertices)}")
    print(f"GT mesh triangles: {len(gt_mesh.triangles)}")


    # ============================================================
    # 3. 计算扫描点到 GT mesh 的距离
    # ============================================================
    print("\n========================================")
    print("Extracting scan points belonging to GT wall...")
    print("========================================")

    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(gt_mesh)

    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh_t)

    query_points = o3d.core.Tensor(
        scan_points,
        dtype=o3d.core.Dtype.Float32
    )

    scan_to_mesh_distance = scene.compute_distance(query_points).numpy()


    # ============================================================
    # 4. 提取 GT 墙面附近的扫描点
    # ============================================================
    wall_mask = scan_to_mesh_distance < association_threshold

    wall_points = scan_points[wall_mask]
    wall_mesh_distance = scan_to_mesh_distance[wall_mask]

    if len(wall_points) == 0:
        raise ValueError(
            "No scan points were associated with the GT wall. "
            "Try increasing association_threshold."
        )

    print(f"Association threshold: {association_threshold:.3f} m")
    print(f"Extracted wall point number: {len(wall_points)}")
    print(
        f"Percentage of total scan: "
        f"{len(wall_points) / len(scan_points) * 100:.2f}%"
    )


    # ============================================================
    # 5. 保存提取出来的墙面
    # ============================================================
    wall_pcd = o3d.geometry.PointCloud()
    wall_pcd.points = o3d.utility.Vector3dVector(wall_points)

    # o3d.io.write_point_cloud(
    #     save_wall_path,
    #     wall_pcd
    # )

    print(f"Extracted wall saved to: {save_wall_path}")


    # ============================================================
    # 6. 使用 GT mesh 顶点拟合真实参考平面
    # ============================================================
    print("\n========================================")
    print("Fitting GT reference plane...")
    print("========================================")

    gt_center = np.mean(gt_vertices, axis=0)

    centered_vertices = gt_vertices - gt_center

    _, _, vh = np.linalg.svd(centered_vertices)

    # 最小奇异值对应方向就是平面法向量
    normal = vh[-1]

    normal = normal / np.linalg.norm(normal)

    d = -np.dot(normal, gt_center)

    # 为了显示方便，如果法向量最大的分量是负数，则统一翻转
    max_axis = np.argmax(np.abs(normal))

    if normal[max_axis] < 0:
        normal = -normal
        d = -d

    print(
        f"GT Plane: "
        f"{normal[0]:.6f}x + "
        f"{normal[1]:.6f}y + "
        f"{normal[2]:.6f}z + "
        f"{d:.6f} = 0"
    )

    print(
        "GT plane normal: "
        f"[{normal[0]:.6f}, "
        f"{normal[1]:.6f}, "
        f"{normal[2]:.6f}]"
    )


    # ============================================================
    # 7. 扫描点到 GT 平面的 signed distance
    # ============================================================
    signed_distance = wall_points @ normal + d

    abs_plane_distance = np.abs(signed_distance)


    # ============================================================
    # 8. Flatness / Surface deviation 指标
    # ============================================================
    mean_dev = np.mean(abs_plane_distance)

    rmse_dev = np.sqrt(
        np.mean(abs_plane_distance ** 2)
    )

    median_dev = np.median(abs_plane_distance)

    p95_dev = np.percentile(
        abs_plane_distance,
        95
    )

    max_dev = np.max(abs_plane_distance)

    std_dev = np.std(abs_plane_distance)

    signed_min = np.min(signed_distance)
    signed_max = np.max(signed_distance)


    # ============================================================
    # 9. 扫描点到 GT mesh 的误差
    # ============================================================
    mesh_mean = np.mean(wall_mesh_distance)

    mesh_rmse = np.sqrt(
        np.mean(wall_mesh_distance ** 2)
    )

    mesh_p95 = np.percentile(
        wall_mesh_distance,
        95
    )

    mesh_max = np.max(wall_mesh_distance)


    # ============================================================
    # 10. 计算 GT 墙面的 Coverage
    # ============================================================
    print("\n========================================")
    print("Computing coverage...")
    print("========================================")

    gt_sample_pcd = gt_mesh.sample_points_uniformly(
        number_of_points=gt_sample_num
    )

    gt_to_scan_distance = np.asarray(
        gt_sample_pcd.compute_point_cloud_distance(
            wall_pcd
        )
    )

    coverage_mask = (
        gt_to_scan_distance < coverage_threshold
    )

    coverage = np.mean(coverage_mask)

    covered_num = np.sum(coverage_mask)


    # ============================================================
    # 11. 输出 GT wall 范围
    # ============================================================
    gt_min = np.min(gt_vertices, axis=0)
    gt_max = np.max(gt_vertices, axis=0)

    wall_min = np.min(wall_points, axis=0)
    wall_max = np.max(wall_points, axis=0)


    # ============================================================
    # 12. 打印结果
    # ============================================================
    print("\n")
    print("========================================")
    print("            Evaluation Result")
    print("========================================")

    print("\n--- Scan Information ---")

    print(
        f"Total scan points: "
        f"{len(scan_points)}"
    )

    print(
        f"Wall scan points: "
        f"{len(wall_points)}"
    )


    print("\n--- GT Wall Geometry ---")

    print(
        f"GT Min: "
        f"[{gt_min[0]:.3f}, "
        f"{gt_min[1]:.3f}, "
        f"{gt_min[2]:.3f}]"
    )

    print(
        f"GT Max: "
        f"[{gt_max[0]:.3f}, "
        f"{gt_max[1]:.3f}, "
        f"{gt_max[2]:.3f}]"
    )

    print(
        f"X range: "
        f"{gt_max[0] - gt_min[0]:.3f} m"
    )

    print(
        f"Y range: "
        f"{gt_max[1] - gt_min[1]:.3f} m"
    )

    print(
        f"Z range: "
        f"{gt_max[2] - gt_min[2]:.3f} m"
    )


    print("\n--- Extracted Scan Wall Range ---")

    print(
        f"Scan Min: "
        f"[{wall_min[0]:.3f}, "
        f"{wall_min[1]:.3f}, "
        f"{wall_min[2]:.3f}]"
    )

    print(
        f"Scan Max: "
        f"[{wall_max[0]:.3f}, "
        f"{wall_max[1]:.3f}, "
        f"{wall_max[2]:.3f}]"
    )


    print("\n--- Coverage ---")

    print(
        f"Coverage threshold: "
        f"{coverage_threshold * 1000:.1f} mm"
    )

    print(
        f"GT samples: "
        f"{gt_sample_num}"
    )

    print(
        f"Covered samples: "
        f"{covered_num}"
    )

    print(
        f"Coverage: "
        f"{coverage * 100:.2f}%"
    )


    print("\n--- Flatness / Point-to-GT-Plane Deviation ---")

    print(
        f"Mean deviation: "
        f"{mean_dev * 1000:.2f} mm"
    )

    print(
        f"Median deviation: "
        f"{median_dev * 1000:.2f} mm"
    )

    print(
        f"RMSE deviation: "
        f"{rmse_dev * 1000:.2f} mm"
    )

    print(
        f"Std deviation: "
        f"{std_dev * 1000:.2f} mm"
    )

    print(
        f"P95 deviation: "
        f"{p95_dev * 1000:.2f} mm"
    )

    print(
        f"Max deviation: "
        f"{max_dev * 1000:.2f} mm"
    )

    print(
        f"Signed min deviation: "
        f"{signed_min * 1000:.2f} mm"
    )

    print(
        f"Signed max deviation: "
        f"{signed_max * 1000:.2f} mm"
    )


    print("\n--- Scan-to-GT Mesh Error ---")

    print(
        f"Mean mesh distance: "
        f"{mesh_mean * 1000:.2f} mm"
    )

    print(
        f"RMSE mesh distance: "
        f"{mesh_rmse * 1000:.2f} mm"
    )

    print(
        f"P95 mesh distance: "
        f"{mesh_p95 * 1000:.2f} mm"
    )

    print(
        f"Max mesh distance: "
        f"{mesh_max * 1000:.2f} mm"
    )


    # ============================================================
    # 13. 返回结果
    # ============================================================
    results = {
        "total_scan_points": len(scan_points),
        "wall_points": len(wall_points),

        "coverage": coverage,

        "mean_deviation": mean_dev,
        "median_deviation": median_dev,
        "rmse_deviation": rmse_dev,
        "std_deviation": std_dev,
        "p95_deviation": p95_dev,
        "max_deviation": max_dev,

        "signed_min_deviation": signed_min,
        "signed_max_deviation": signed_max,

        "mesh_mean_distance": mesh_mean,
        "mesh_rmse_distance": mesh_rmse,
        "mesh_p95_distance": mesh_p95,
        "mesh_max_distance": mesh_max,

        "plane_normal": normal,
        "plane_d": d
    }

    return results


if __name__ == "__main__":

    results = evaluate_wall_scan(

        # scan_pcd_path="map_imad1_room1.pcd",
        scan_pcd_path="map_voxel_tare_room1_wall1.pcd",

    
        gt_mesh_path="indoor_surface.obj",

        # 点云距离 GT mesh 10 cm 内认为属于该墙
        association_threshold=0.03,

        # GT 表面 5 cm 内存在扫描点认为被覆盖
        coverage_threshold=0.05,

        # GT mesh 表面采样数量
        gt_sample_num=200000,

        save_wall_path="extracted_wall.pcd"
    )