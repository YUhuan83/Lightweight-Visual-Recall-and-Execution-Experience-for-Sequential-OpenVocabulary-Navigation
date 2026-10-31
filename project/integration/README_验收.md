# 端到端集成验收（同步采集 → 开放词汇查询 → 严格校验 → 多帧定位）

## 架构

```
capture_frames.py          run_e2e.py                    target_localization
(beamngpy 环境)            (vision 环境)                  (仅 numpy)
      │                          │                            │
      ▼                          ▼                            ▼
 fixture 场景 ──► frames/    vocabulary_query/query.py ──► load_query ──► Localizer
 相机+雷达        image.png     (vision 环境, 真实推理)      (严格校验)   (3 帧确认)
 冻结同步采集     cloud.npy
                 capture.json
```

## 模块

| 模块 | 作用 | 依赖 |
|---|---|---|
| `capture_frames.py` | 暂停→单步→冻结，同步采集相机+雷达帧 | beamngpy, sensor_calibration |
| `common.py` | 夹具几何（标称 K、静态 SE3）与采集读取 | numpy |
| `make_calibration.py` | 生成 `target-calibration/1`（默认 DRAFT） | numpy |
| `build_frame.py` | 构建 `target-frame/1`；不同步则拒绝 | numpy |
| `run_e2e.py` | 端到端驱动（查询→校验→定位） | vocabulary_query, target_localization |

## 已验证

- 本地测试 **15 项全通过**（标称焦距、SE3 正交、棋盘投影、DRAFT/VERIFIED、同步检测、严格 frame 契约、采集记录合成、**3 帧独立确认→CONFIRMED**）。
- 用真实夹具采集 + 服务器查询结果实测：4 条查询全部通过严格 `load_query`；图像/雷达相差 0.957 s，定位阶段**如实拒绝**，退出码 2。

```powershell
python -m unittest integration.test_integration -v
```

## 服务器运行

### 1. 上传源码（本地 PowerShell）

```powershell
scp -r "C:\Users\Yu\Desktop\拟真\integration" ly@192.168.31.91:~/vision_query/
scp -r "C:\Users\Yu\Desktop\拟真\target_localization" ly@192.168.31.91:~/vision_query/
```

### 2. 同步采集（beamngpy 环境，需 BeamNG 能渲染）

```bash
cd /home/ly/vision_query
~/venvs/beamng-039/bin/python -B integration/capture_frames.py \
  --home <BeamNG Linux home> --user <userpath 父目录> --replace-current \
  --frames 3 --stride 0.5 --gfx vk \
  --output ~/capture_out
```

产出 `~/capture_out/<runid>/`：
- `frames/frame_NNN/{image.png, cloud.npy, capture.json}`
- `capture_report.json`

`capture.json` 为 schema `capture-frame/1`，含：共享采集时间戳、相机/雷达位姿、`world_to_camera` SE3、冻结校验 `pose_stability`、`synchronized` 布尔。任一传感器或车辆在采集期间移动，则该帧 `synchronized=false`。

### 3. 端到端（vision 环境）

```bash
cd /home/ly/vision_query
~/vision-cu128/bin/python -m integration.run_e2e \
  --frames-dir ~/capture_out/<runid>/frames \
  --query "black white checkerboard" \
  --source-root /home/ly/wildos-sim/wildos_port/repro_check/src/nebula2-wildos \
  --checkpoint-root /home/ly/ckpts \
  --query-script /home/ly/vision_query/vocabulary_query/query.py \
  --query-python /home/ly/vision-cu128/bin/python \
  --output ~/e2e_out
```

每帧独立跑查询（不同图像=不同确认）。3 帧通过严格校验且目标一致时，报告 `stages.localization.confirmed == 1`、`states` 含 `CONFIRMED`。

### 4. 旧格式采集仍支持

```bash
--capture ~/fixture_capture        # 旧的 mounted/static/lidar 清单
--reuse-query <已有查询目录>        # 仅 --capture 模式可用，跳过推理
```

## 输出解读

`integration_report.json`：

- `mode`：`multi_frame` 或 `legacy_capture`
- `stages.calibration`：`DRAFT` 或 `VERIFIED`
- `stages.localization`：`multi_frame` 下含 `states`、`confirmed`；`legacy_capture` 下含 `refused`/结果
- `regions` / `frames[].regions`：候选区域框与余弦统计
- `blockers`：阻止定位的精确原因列表

## 实测：3 帧确认成功

服务器实测（runid `a221141e`，棋盘查询）：

```
states = [TENTATIVE, TENTATIVE, CONFIRMED]   confirmed = 1
frame_002 CONFIRMED confirmation_count=3 reason=three_frame_evidence
position_m ≈ (-0.067, -9.999, 2.39)    # 夹具棋盘中心 (0, -9.999, 2.35)
```

链路：同步采集（暂停+单步，冻结已校验）→ 逐帧真实模型查询 → 严格 `load_query` → 夹具标定（VERIFIED）→ 3D 投影 → 3 帧时序确认。

**重要说明**：该夹具目标需要显式放宽策略才能通过（见 `run_cb.sh`）：
- `--max-extent-m 8 --max-depth-span-m 3.5`：棋盘 5×3.5m，比默认 4m/1.5m 大。
- `--max-ring-same-depth-fraction 0.99 --alpha-silhouette 0.01`：棋盘紧贴背板，与背景无深度差；默认策略会**正确地**拒绝为 `background_or_unseparated_depth`。
- `--ttl-ns 30000000000`：采集帧相隔约 2s，默认 1s 证据窗口会在下一帧前过期。

这些是**夹具专用**放宽，用于验证管线；默认严格策略不变。真实障碍物（凸出、有背景深度差）无需这些放宽。

## 已知限制

1. **同步依赖冻结期间 ad-hoc 渲染**：若渲染器在暂停时无法出图，采集会超时并如实报错，不产出伪同步数据。`--stride 0` 的各帧视角相同，只能算一次独立确认。
2. **标定默认 DRAFT**：`api_fit` 焦距为负（约定符号问题），幅值与标称 554.256 仅差 0.4%。需显式传 `--accept-calibration "<证据>"` 才标 VERIFIED。
3. **模型颜色分辨率弱**：合成纯色块场景不宜用颜色文字查询（红/蓝/绿 cube 相互混淆）；验证锚点应改用棋盘等结构目标。
4. **车辆移动仍未解决**：本脚本用重建静态传感器 + `--stride` 横向平移取得不同视角，不依赖车辆行驶。
