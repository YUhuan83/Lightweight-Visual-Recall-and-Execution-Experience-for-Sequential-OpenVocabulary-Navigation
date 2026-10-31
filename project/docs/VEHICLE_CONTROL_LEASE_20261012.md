# 车端控制过期制动：2026-10-12

## 已确认的问题

上一轮 `direct_steering_short_curve1` 中，执行 RPC 超时后客户端永久关闭执行连接，虽然发布了 `watchdog_stop` 和制动意图，BeamNG 实际刹车仍为 0、速度仍超过 0.5 m/s。独立新连接制动才恢复停车。旧失败记录保持原样，入口为 `docs/DIRECT_STEERING_REPAIR_20261012.md`。

客户端超时保护只能阻止后续发送，不能撤销已留在车端的油门。上轮健康门控阻止失效后的完成和经验学习，本轮补上车端控制有效期。

## 实现和接口

- `nav2_adapter/wildos_nav2_adapter/vehicle_control_lease.lua`：车辆 Lua 扩展，用实际安装的 `HighPerfTimer` 墙钟检查最后有效指令年龄。原有 0.8 秒 `command_timeout` 同时作为车端有效期，没有扩大保护阈值。
- `vehicle_control_lease.py`：安装前比较完整源码，拒绝覆盖未知版本；加载后的源码 SHA 和版本必须一致。存在检查使用 `rawget`，避免 BeamNG 扩展表的自动加载副作用。Lua 作为包资源随 wheel 发布。
- `direct_vehicle_control.py`：每个 Native 实例有一个新 owner 和递增 sequence。一次车端请求先验证有效期、owner 和 sequence，再设置输入，ACK 包含 owner/sequence。迟到、重复、乱序及旧 owner 请求不能续命。诊断用的旧无租约 helper 保留原接口，生产 Native 必须使用租约。
- `native_vehicle_control.py`：取得控制权后再关闭 AI 和设置换挡模式。执行连接失效仍不自动重连；停止时可通过现有遥测连接释放自身控制权，即使最后的制动请求失败也会尝试释放，并保留原异常。

有效指令仍使用原设置：转向 DIRECT，其他输入 PAD，原挡位逻辑。车端过期后，普通输入置为零油门、全制动和驻车制动，保持制动；不修改车身、轮胎、质量、碰撞、hydros 或 8 米转弯半径。GFX 和 physics 回调均检查过期；physics 回调按 0.02 秒模拟时间间隔检查，过期依据仍是墙钟。

旧 owner 过期后不能自行恢复。新 owner 只能在实际速度低于 0.05 m/s 时申请；活动的另一 owner 被拒绝。移动中释放保持制动守护，静止释放才解除。重置会使旧 owner 失效。`status()` 是只读函数，不能替代回调触发制动。

## 验证范围

| 验证 | 结果 | 范围 |
|---|---|---|
| 本机完整回调 runner | 307 通过 | ROS 接口替身，无 DDS/车辆 |
| 服务器相同 runner | 307 通过 | 同样为替身，不与本机相加 |
| 实际 VE Lua VM | 14 通过 | 假输入/速度/时钟隔离测试，无车辆控制 |
| wheel 构建和资源核对 | 通过 | Lua 文件实际包含在 wheel 中 |
| stationary2 | 通过 | 真实 SBR 全制动，回调自行过期，迟到输入被拒绝，新 owner 可申请/释放 |
| freshness1 | 五阶段通过 | 真实 BeamNG + DDS，650 ms ACK/遥测延迟及恢复；只有全制动输入 |
| short_curve1 | 通过 | 实际 CarLike、GraphPathBridge、FollowPath；5.02375 m 安全曲线，移动 4.54211 m，终点误差 0.558157 m，持续停稳 |
| moving_fault2 | 通过 | 真实规划路径运动中挂起本轮 Native，车端过期停车，恢复进程后拒绝旧 owner，桥保持 execution_unavailable |
| 本机证据检查 | 6 通过 | 冻结源码、归档哈希、实际数据及失败记录一致性 |

`moving_fault2` 挂起控制进程后的首个独立采样速度为 1.61376 m/s、实际油门为 0.02710、刹车为 0。0.782008 秒观察到租约过期、实际全制动。66 个故障停车采样中最大速度 2.03953 m/s，最后速度 0.00425059 m/s，并持续低于 0.05 至少 0.5 秒。读取租约状态及任何退出制动均发生在实际停车之后。故障后总位移为 1.82539 m，不能把 0.8 秒有效期理解成零停车距离。

恢复 Native 后新鲜遥测年龄 0.004764 秒、速度 0.0088214 m/s、实际油门 0/刹车 1；旧 owner 输入被车端拒绝，ACK 和 actuator_ok 为 false。桥出现 `execution_unavailable`，没有 `route_done`。这是故障停车与终态交接验收，没有视觉到达或记忆检索。

## 保留的失败和更正

1. `stationary1.json`：只传输 Lua/Python 安装模块时，服务器旧 direct helper 不支持 owner 参数，测试失败且没有运动。之后部署完整冻结候选再运行 stationary2。未知源码不会被覆盖。
2. `server_related.log`：误将依赖 ROS 替身的回调测试直接放进真实 ROS 类型环境，35 项中 8 失败、9 错误，包含未初始化 Node。保留该运行；使用正式隔离 runner 后服务器 307 项通过，不通过修改生产代码迁就测试启动错误。
3. `moving_fault1`：脚本结果原为 true，但退出停车记录覆盖了故障停车数组，恢复记录也只是明确标记为过期的缓存遥测。`diagnostic_review.json` 明确记为证据验收失败；原记录不改写。moving_fault2 分离故障/退出数组，并要求新鲜恢复遥测后才通过。
4. 本机首次 pip wheel 的默认临时目录受到权限限制；改用项目内临时目录、无网络构建通过。wheel SHA 为 `0003801724bc103bc403c60f7d97e46e76636d116bd06a1ddc361711654cd750`。

## 证据和部署

- 本机归档：`C:\Users\Yu\Desktop\拟真\test_artifacts\server_validation_20261012\vehicle_control_lease_evidence.zip`。
- 服务器原始目录：`/home/ly/town_memory_validation_20261012/vehicle_control_lease/`。
- 归档 795309 字节，118 个原始成员；SHA256 `25f108cd01452c611030f8b5cdb8561eef9c5b2373a6392ee249d982c3c84c07`。本机逐成员核验，并一次核对本机 10 个部署文件与服务器冻结清单一致。
- 车端实际加载源码 SHA `a02898bdd7bfa94458a2b5447d75797b3bd8404b9fdb10a7f2f024f0bfc16377`，最终 owner 已释放。
- 候选位于 `/home/ly/town_memory_validation_20261009/physical_profile_fix/candidate`。原生产目录仍干净，HEAD `f6d7ab9602911d871ffd8e33ecb4f20c0d66feb0`。
- boot_id 未变；BeamNG 422172、Xvfb 422169 保留，同地图、同车辆 16266。0 重载/0 瞬移/0 视频。最终实际速度 0.00570656 m/s、仿真暂停、传感器和本轮进程均已清理。723 节点、0 固定节点、467 车体碰撞节点、四轮各 32 碰撞节点保持。共享磁盘可用约 25.41 GB。

回退前必须停止所属执行进程并确认实际停稳、租约已释放；不能在活动租约仍持有制动时直接启动旧控制客户端。修复前 Native/direct 文件保存在归档的 `before`，完整部署前备份在 `before_complete`。源码固定检查拒绝未知文件；未来升级或回退车端 Lua 必须在释放控制权后明确处理版本，不静默替换活动模块。

## 仍未解决

本测试挂起的是本轮控制进程，BeamNG 本身仍运行，不能证明引擎全部回调冻结时可立即制动。全部回调冻结时 Lua 也无法执行；恢复后的迟到请求保护已由隔离状态测试覆盖，真实引擎冻结/恢复尚未注入验收。

共同引擎请求数秒空档及历史 native 崩溃的具体来源仍未查明。独立 costmap 退出故障、复杂路径 excess_curvature 来源、长曲线/cusp 执行及完整目标搜寻仍需继续。正常短曲线通过不证明这些问题消失，也不构成绿色房屋或视觉记忆成功。

完整绿色房屋 pilots1–5 的失败仍保留；20 轮/80%及多目标均未开始。本次组件验证已通过，必要完整验收仍待进行，按用户规则不 commit，目标继续活动。
