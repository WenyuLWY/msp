import open3d as o3d
import numpy as np

def compute_recall(gt_pcd, scan_pcd_path, distance_thresholds=0.152):
    """
    计算扫描点云相对于真值 (Mesh 或 点云) 的 Recall (Completeness)
    
    :param gt_path: 真值文件路径 (.mesh, .ply, .pcd 等)
    :param scan_pcd_path: 待评估的扫描点云路径 (.pcd, .ply 等)
    :param distance_thresholds: 判定为“已覆盖”的距离阈值列表（单位与点云一致，通常为米，如 1cm, 2cm, 5cm）
    :param num_gt_samples: 若真值为 Mesh, 采样出的真值点云数量
    """
    # 1. 加载待评估的扫描点云
    scan_pcd = o3d.io.read_point_cloud(scan_pcd_path)
    if not scan_pcd.has_points():
        raise ValueError("无法读取扫描点云或点云为空！")
    
    # 2. 加载真值 (判断是 Mesh 还是 Point Cloud)
    # 尝试作为 Mesh 读取
    # gt_mesh = o3d.io.read_triangle_mesh(gt_path)
    
    # if len(gt_mesh.triangles) > 0:
    #     print("检测到真值为 Mesh, 正在进行表面均匀采样...")
        # 为了保证面积均匀性，使用 Poisson Disk Sampling 或 Uniform Sampling
        # gt_pcd = gt_mesh.sample_points_poisson_disk(number_of_points=num_gt_samples)
        # gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=num_gt_samples)
    # else:
    #     # 若不是 Mesh，作为点云读取
    #     gt_pcd = o3d.io.read_point_cloud(gt_path)
    #     if not gt_pcd.has_points():
    #         raise ValueError("无法读取真值文件，请检查文件路径和格式！")
    #     print("检测到真值为点云。")


    scan_distances = scan_pcd.compute_nearest_neighbor_distance()
    scan_distances = np.asarray(scan_distances)

    print("Point number:", len(scan_pcd.points))
    print(f"Mean spacing: {np.mean(scan_distances):.2f}")
    print(f"Median spacing: {np.median(scan_distances):.2f}")
    # print(f"Std spacing: {np.std(scan_distances):.2f}")
    # print(f"P95 spacing: {np.percentile(scan_distances, 95):.2f}")

    gt_distances = gt_pcd.compute_nearest_neighbor_distance()
    gt_distances = np.asarray(gt_distances)

    print("GT Point number:", len(gt_pcd.points))
    print(f"GT Mean spacing: {np.mean(gt_distances):.2f}")
    print(f"GT Median spacing: {np.median(gt_distances):.2f}")
    # print(f"GT Std spacing: {np.std(gt_distances):.2f}")
    # print(f"GT P95 spacing: {np.percentile(gt_distances, 95):.2f}")

    # 3. 构建 KD-Tree 用于加速近邻查找
    # 我们关注的是 GT 点到扫描点云的最近距离
    scan_kdtree = o3d.geometry.KDTreeFlann(scan_pcd)
    
    gt_points = np.asarray(gt_pcd.points)
    num_gt_points = len(gt_points)
    
    # 4. 计算每个 GT 点到扫描点云最近点的距离
    distances = np.zeros(num_gt_points)
    for i in range(num_gt_points):
        # search_knn_vector_3d 返回: (k, idx, dist_squared)
        _, _, dist_sq = scan_kdtree.search_knn_vector_3d(gt_points[i], 1)
        distances[i] = np.sqrt(dist_sq[0])


    count = 0
    threshold = 0.3

    for point in scan_pcd.points:
        [k, idx, dist2] = scan_kdtree.search_knn_vector_3d(point, 2)
        if k >= 2:
            nearest_distance = np.sqrt(dist2[1])
            if nearest_distance < threshold:
                count += 1
    print("Point number:", len(scan_pcd.points))
    print("Points with nearest-neighbor distance:", count)
    print(f"Ratio: {count / len(scan_pcd.points) * 100:.2f}%")

    # 5. 根据不同阈值计算 Recall / Completeness
    print("\n========= 评估结果 (Completeness / Recall) =========")
    print(f"真值参考点数: {num_gt_points}")
    print(f"扫描点云点数: {len(scan_pcd.points)}")
    print("-----------------------------------------------------")
    
    # recalls = {}
    # for th in distance_thresholds:
    #     # 距离小于阈值 th 的 GT 点被认为是成功覆盖的
    #     covered_points_count = np.sum(distances < th)
    #     recall = covered_points_count / num_gt_points
    #     recalls[th] = recall
    #     print(f"Recall @ {th*100:.1f} cm : {recall * 100:.2f}% ({covered_points_count}/{num_gt_points})")

    covered_points_count = np.sum(distances <= distance_thresholds)
    recall = covered_points_count / num_gt_points
    print(f"Recall @ {distance_thresholds*100:.1f} cm : {recall * 100:.2f}% ({covered_points_count}/{num_gt_points})")


    print(f"平均距离误差 (Mean Distance): {np.mean(distances):.4f} m")
    print(f"中位数距离误差 (Median Distance): {np.median(distances):.4f} m")
    print("=====================================================\n")
    
    return gt_pcd, distances, recall

if __name__ == "__main__":
    # 替换为你的文件路径
    # GT_FILE = "pointcloud.ply"  # 可以是 .ply, .obj Mesh 或 .pcd 点云
    GT_FILE = "indoor.obj"  # 可以是 .ply, .obj Mesh 或 .pcd 点云
    SCAN_FILE1 = "indoor300/map_voxel_tare.pcd"
    SCAN_FILE2 = "indoor300/map_voxel.pcd"
    SCAN_FILE3 = "indoor300/map_imad.pcd"

    # gt_mesh = o3d.io.read_triangle_mesh(GT_FILE)
    # gt_pcd = gt_mesh.sample_points_uniformly(number_of_points=5000)
    # gt_distances = gt_pcd.compute_nearest_neighbor_distance()
    # gt_distances = np.asarray(gt_distances)
    # print("GT Point number:", len(gt_pcd.points))
    # print(f"GT Mean spacing: {np.mean(gt_distances):.3f}")
    # print(f"GT Median spacing: {np.median(gt_distances):.3f}")
    
    # 阈值设置：如 1cm (0.01m), 3cm (0.03m), 5cm (0.05m)
    # THRESHOLDS = [0.075, 0.1, 0.15] 
    THRESHOLDS = 0.3

    gt_pcd = o3d.io.read_point_cloud("pointcloud.ply")
    # scan_pcd = o3d.io.read_point_cloud(SCAN_FILE1)
    # gt_pcd.paint_uniform_color([0.0, 0.0, 0.0])
    # scan_pcd.paint_uniform_color([0.0, 1.0, 0.0])
    # o3d.visualization.draw_geometries(
    #     [gt_pcd, scan_pcd],
    #     window_name="GT vs Scan Point Cloud",
    #     width=1280,
    #     height=720
    # )

    gt_pcd, dists, recall = compute_recall(
        gt_pcd=gt_pcd, 
        scan_pcd_path=SCAN_FILE1, 
        distance_thresholds=THRESHOLDS,
    )

    gt_pcd, dists, recall = compute_recall(
        gt_pcd=gt_pcd, 
        scan_pcd_path=SCAN_FILE2, 
        distance_thresholds=THRESHOLDS,
    )
    gt_pcd, dists, recall = compute_recall(
        gt_pcd=gt_pcd,  
        scan_pcd_path=SCAN_FILE3, 
        distance_thresholds=THRESHOLDS,
    )


