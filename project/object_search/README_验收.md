# 开放词汇目标粗定位（WildOS 官方算法的独立实现）

## 来源与对齐

本模块是 WildOS（arXiv 2602.19308）中"粗目标定位"的**无 ROS 独立移植**，数学与官方源码逐条对齐：

| 本模块 | 官方来源 |
|---|---|
| `Ray.distance_to_points` | `triangulation3d/camera_data.py` |
| `ray_from_bbox` | `triangulation3d/bbox_generator.py` |
| `generate_particles` | `triangulation3d/particle_generator.py` |
| `triangulate` | `triangulation3d/triangulator.py`（逆距离射线加权均值）|
| `localize_in_range` | `obj_mask_triangulation.py check_lidar_in_mask`（掩码内点云中位深度）|

约定：`K` 标准针孔（x 右、y 下、z 前）；`(R_wc, t_wc)` 为**相机→世界**，`p_world = R_wc·p_cam + t_wc`。

## 论文对应

- **目标在雷达范围内**：取掩码内点到相机**欧氏距离的中位数** → 世界坐标（论文一句带过的平凡情形）。
- **目标在雷达范围外（论文核心贡献）**：每视图从掩码内随机采像素、深度 `~U(dmin,dmax)` 反投影成 3D 粒子；对所有视图的"掩码质心射线"，按粒子到各射线距离的**倒数和**加权，求加权均值作为粗目标。

## 测试

```
python -m unittest object_search.test_triangulation -v
```

7 项：射线距离（在射线上=0、原点后=∞）、bbox 射线朝向、范围内中位深度恢复、点多不足返回 None、`localize` 优先范围内、两视角粒子三角化 0.6m 内恢复已知目标。

## 服务器运行（用真实采集帧 + 查询掩码）

需先有 `integration.capture_frames` 采集的帧，以及 `integration.run_e2e` 产生的逐帧查询目录（`vocabulary_query/integration_<epoch>_frame_NNN/`）。

```bash
cd ~/vision_query
~/vision-cu128/bin/python -m object_search.run_object_search \
  --frames-dir ~/capture_out/<runid>/frames \
  --vocab-dir ~/vision_query/vocabulary_query \
  --epoch <runid> --prefix query_000 \
  --calibration ~/e2e_out/calibration.json \
  --output ~/object_search_out
```

输出 `object_search.json`：每帧范围内 LiDAR 定位 `lidar_lock`，以及跨帧粒子三角化 `triangulated_position`。

## 深度先验与收敛（关键机制）

官方的范围外三角化**深度几乎完全由采样先验决定**，几何只约束横向。本地参数扫描（三视角、4m 基线、真值 z=5）：

| depth_range | 恢复 z | 误差 |
|---|---|---|
| (1, 100) 官方默认 | 22.07 | 17.07 |
| (4, 6) | 4.95 | 0.05 |
| (1, 10) | 4.15 | 0.85 |

因此官方提供 `get_gaussian_samples`：给定一个 2D/3D **深度假设**，在假设深度附近采样（`gaussian_hyp`/`uniform_hyp`，`hypothesis_std` 默认 5m），估计随即收敛。真实数据上也可复现：不加假设 `y≈-24.6`（真值 -10），加假设 `--hypothesis-xyz 0 -10 2.35 --distribution gaussian_hyp --hypothesis-std 1.0` 后收敛到约 -10。

```bash
bash ~/run_objectsearch.sh e45b06b2 \
  --hypothesis-xyz 0 -10 2.35 --distribution gaussian_hyp --hypothesis-std 1.0
```

## 边界与限制

- 采集帧视角接近（±0.5m 偏移）且目标在雷达范围内，因此真实数据主要验证**范围内**路径；**范围外粒子滤波**目前仅由合成两视角测试验证，尚无远距多视角真实数据。
- 与官方一致：范围外结果是**粗定位**，深度均匀采样，随目标进入雷达范围自动转为精确中位深度。
- 本模块只估计候选目标位置，**不接运动控制**（与官方 `/wildos_port/candidate_goal` 同一边界）。
