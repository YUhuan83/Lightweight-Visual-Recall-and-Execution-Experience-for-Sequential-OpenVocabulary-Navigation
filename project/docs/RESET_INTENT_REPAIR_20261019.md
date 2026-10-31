# 清理事务预算修复：组件通过，整车待验收

## 为什么会长期静止

6153冻结版的164222完整测试中，末段约132秒持续静止与 `graph_reset_failed / response_budget_exhausted` 关联。图仍发布路线，但Bridge的 `query_reset_pending` 屏障阻止接受。源码反例表明：初始清理及多次视觉释放的成功请求共用累计计数，第三次释放后可能不再派发清理。不是三次真实服务超时，也不是这132秒的Native通信延迟。

这不是所有停车的唯一原因。独立真实车体短段已复现持续车身接触、正常驱动却无进展；视觉超时清除物理恢复状态、C++ clear_goal清探索回退历史等问题仍开放。地图与恢复交接必须一起处理。

## 本次实现

- `navigation_transactions.py::request_graph_reset`：同query generation及确认fence下，尚未完成的重复清理请求合并为同一事务，不抛弃当前响应归属，也不补充失败预算；完成后再次请求、新scope请求各有新预算。
- `visible_approach.py::release_visible_approach`：通过统一入口申请清理，不再直接设置pending/inflight而继承旧计数。
- 保留最多三次派发、旧请求排空、旧响应隔离、实例释放屏障、未知动作取消及停稳保护。没有放宽源寿命、碰撞或物理限制。
- 尚未改变视觉释放的通用reset行为，不能声称脱困状态丢失也已解决。

## 反例及验证

本机证据：`test_artifacts/reset_intent_repair_20261019/`。

| 验证 | 结果与范围 |
| --- | --- |
| 原实现四个新反例 | 3失败、1通过；before.json/log及原源码保留 |
| 新实现事务回归 | 41通过，含多次成功释放、重复请求、无响应预算、新scope及旧响应排空 |
| 全部离线回调 | 551通过，ROS替身，不是实际车辆/DDS动作服务器 |
| 首次服务器真实ROS接口回调 | 41项中5夹具错误、0断言失败；真实发布方法不能用Mock断言，真实Clock无替身steady属性。失败日志保留 |
| 修正测试注入后真实ROS接口回调 | 41通过；显式Mock发布器，Clock包装真实构造并检查STEADY_TIME参数；未删测试或放宽生产保护 |

真实ROS回归仍注入服务/动作响应，没有真实DDS动作服务器或BeamNG整车，`complete_navigation_passed=false`。本次没有新增搜索、录像、稳定性轮次，也没有Git commit。

## 版本与部署边界

- 原6153归档及服务器正式候选保持不变。本机两份实现现在有意偏离该冻结版，不得再声称当前源码全部404成员仍相同。
- 服务器独立目录 `/home/ly/reset_intent_repair_20261019/candidate_portable`：先核对6153的404成员，再叠加四文件增量，ROS域192。
- navigation_transactions实现SHA：`da57b81218172b552fe415e8589b1d6a5cd3a509c81c71d3883f149905dfbebe`。
- visible_approach实现SHA：`fd5cdd2e605ae250cba768417057ad84305ebcf72983f1a3f5a38a6c4f13ee99`。
- 本机/服务器实际导入路径与SHA均在报告中核验。初次失败candidate及便携夹具candidate_portable分别保留，未覆盖失败记录。
- 测试结束后的独立进程检查为空；GPU1其他负载未触碰。

后续先修清理/视觉/物理恢复各自归属与服务副作用，再与可走空间修复共同验收。组件通过不等于长期静止已在现场消失。
