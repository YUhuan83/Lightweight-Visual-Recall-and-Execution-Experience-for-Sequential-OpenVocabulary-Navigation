# BeamNG → ROS2 → RViz

本次服务器：`192.168.31.91`，ROS2 Humble，工作区 `/home/ly/beamng_ros2_0394_ws`。
当前导航使用 `ROS_DOMAIN_ID=40`、`ROS_LOCALHOST_ONLY=1`；RViz 在服务器显示 `:99`，窗口适配1280×1024。

## 数据链路

BeamNG的StateSensor、相机和LiDAR由原生BeamNG ROS桥发布。`beamng_wildos_adapter`转换成WildOS的官方话题和坐标：

- `/spot1/odom`：后轴观察位姿与速度。
- `/spot1/realsense/front/color/image_raw`：实时前相机。
- `/spot1/ouster/front/points_filtered`：保留世界位置的LiDAR点云。
- `/spot1/local_map`：供GraphNav建图的局部占据栅格。
- `spot1/odom → spot1/base_link`：后轴参考点的三维物理姿态，供传感器使用。
- `spot1/odom → spot1/nav_base_link`：相同世界XY、仅yaw且z=0的后轴导航参考点，供Nav2二维规划和RPP使用。
- `spot1/base_link → spot1/ouster/front/os_lidar`：原虚拟点云参考点到后轴的固定偏移；相机继续使用独立的实测世界TF。

Town的世界高度约52米。二维路径不能直接在带俯仰/侧倾的三维base_link中计算，否则高度差会混入前后/左右误差。Nav2的两个costmap已统一使用`spot1/nav_base_link`。

## RViz显示

配置：`nav2_adapter/config/beamng_navigation.rviz`。固定坐标系`spot1/odom`，视角跟随`spot1/base_link`。

| 内容 | RViz话题 | 显示 |
|---|---|---|
| 前相机、LiDAR | 官方相机/点云话题 | 原始数据 |
| 全局/局部代价地图 | `/wildos/viz/global_costmap`、`/wildos/viz/local_costmap` | 在车辆世界高度附近的显示副本 |
| 车辆足迹 | `/wildos/viz/footprint` | 青色多边形 |
| 全部车式规划路径 | `/wildos/viz/planned_path` | 黄色 |
| GraphNav路线 | `/wildos/viz/graph_path` | 蓝色 |
| RPP当前局部接收路径 | `/wildos/viz/received_plan` | 粉色，将局部二维路径转换到世界坐标后显示 |
| 实际位姿轨迹 | `/wildos/odom_trail` | 绿色 |
| 记忆观察视点 | `/wildos/debug_markers` | 蓝色朝向箭头 |
| 选中的记忆目标 | `/wildos/debug_markers` | 黄色球体 |
| 查询和规划状态 | `/wildos/debug_markers` | 车辆上方文字 |

`visualization_bridge`只订阅导航数据并发布显示副本。它不发布车辆控制、目标搜索命令或导航输入；导航器继续消费原话题。显示副本的高度修正不代表新的地形高程地图。

局部RPP路径的显示转换使用最近收到的车辆位姿，因此快速运动时可能出现少量显示时间偏差；原始路径及控制计算不受影响。

## 新会话启动查看

先启动或连接现有BeamNG ROS/WildOS导航链路，再在服务器环境运行：

```bash
source /opt/ros/humble/setup.bash
source /home/ly/beamng_ros2_0394_ws/.venv/bin/activate
source /home/ly/beamng_ros2_0394_ws/install/setup.bash
export ROS_DOMAIN_ID=40
export ROS_LOCALHOST_ONLY=1
export DISPLAY=:99
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch wildos_nav2_adapter beamng_rviz.launch.py
```

RViz可独立关闭和重开；地图和视觉记忆所在的进程无需随之重启。仅启动RViz不会加载地图或驱动车辆。结束行驶测试时禁用`follow_path`和控制`enable`并暂停仿真，RViz保留最后画面。

## 实车几何配置

此次Town车辆为Hirochi SBR。四个车轮的轮毂节点测得轴距2.500866米；后轴相对StateSensor约`[-2.172, 0.330, 0]`，z保留原测量高度。后轴足迹为前3.4米、后1.05米、左右1.05米，覆盖实际车身并含少量余量。转向上限0.6输入时测得后轴半径约7.407米；规划半径仍为保守的8米。

更换车型必须重新测量参考点、轮轴和足迹，不能直接复用SBR参数。当前GUI链路可用不代表“寻找房屋后返站”的完整任务已经通过，需查看现场测试结果。
