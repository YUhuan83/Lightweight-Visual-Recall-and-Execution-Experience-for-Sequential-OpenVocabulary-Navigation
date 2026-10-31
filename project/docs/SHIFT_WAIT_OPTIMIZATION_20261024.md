# 换向近停独立优化（2026-10-24）

用户要求优先集中优化换向近停，保留持续探索版本对无新鲜命令近停的改善。

## 根因复核

- 已关闭持续探索r2轮211048/211456分别13/12组gear_change，低速<=.15m/s等待积分25.6056/23.3014s。两遍只读审计SHA均b5800e297a6c2609fbdb57b758c5678775400e84a753507b2cfcb3e8cffb8862。该口径<=.15与整轮near-stop<.1不同，不能当作同一精确秒数。
- 大多数单组持续1.95–2.00s；示例起速.120/.042/.047与.037/.041/.00088m/s，已低于换挡阈值，目标方向整组稳定，控制命令剩余寿命约.40–.44s。新挡位直到组开始后约2.06–2.21s才真正发给Native，等待主要发生在软件换挡请求发出之前。
- 实际D/R反馈的示例在新挡位sent后当次或后1–2个Native样本可见（约0–.16s），不是精确机械响应耗时：遥测有采样间隔和年龄，0不代表瞬时切换，缺失者不能算成功。D selector=2而actual gear_index常为1，审计按实际正负行驶挡位解释，不能直接整数相等。
- `planning.py::update_direction_shift`的2秒实际从相反方向请求开始计时，速度<=.15才允许切换，并非严格“停稳后连续再等2秒”。因为这两轮多数组开始已经近停，所以几乎全部软件确认时间落在低速区。规划仍生成前/后退交替，导致此成本被重复13/12次；本次先减少每次等待，不同步改路径选择。

## 最小改动与影响

从冻结r2派生独立v25_fast_shift，只在physical_vehicle_control.launch.py覆盖gear_change_confirmation_time=.3，添加test_physical_shift_profile_ros.py。默认控制器仍2秒，旧r2和主目录源码不覆盖。既有helper、speed<=.15门、命令与状态/Native新鲜度、源ACK、真实车体、Smac/RPP和路径策略不变；不禁止倒车、不扩大速度租约。

.3秒作为物理配置的方向请求去抖候选（20Hz约6周期），不是宣称适用于所有真车的变速箱常数。真实车辆迁移仍需按车辆接口与反馈标定。现场须比较实际换挡后位移/方向与停顿，防止仅用参数和单元测试宣布改善。

## 验证状态

- 新6项覆盖双向稳定切换、短时方向反复、仍在运动时禁止换挡、命令到期、其他输入新鲜时单独命令到期，以及停机打断后重新确认。改前两遍3fail/0error；改后两遍6项全通过。
- 其余完整离线806项通过（shift_full_regression.log）。服务器冻结429成员，2文件增量；构建/真实ROS及两轮180s现场待结果。无录像/commit。两轮效果未通过前不替换已保存r2。

证据位于test_artifacts/continuous_exploration_20261024，审计audit_continuous_shift_wait.py、测试shift_profile_before_1/2.log、shift_profile_after_1/2.log，增量fast_shift_delta.zip；服务器v24_continuous_exploration_r2/shift_wait_audit.json和新v25_fast_shift保留。

## 已关闭两轮结果

两包构建、602项真实ROS回归通过。现场只读GetParameters核实confirmation=.3、direction_change_speed=.15、command/state timeout=.5、require_stamped_command=true。两轮213139/213551启动38.358/38.361s、搜索180.200/180.176s、关闭22.827/21.629s，均timeout未完整到达、session errors=[]、console_exit=0；结束后项目进程空/X148锁无，不录视频、不commit。

| 指标 | 改前r2两轮 | .3秒候选两轮 |
|---|---|---|
| gear_change近停<.1m/s | 25.12 / 23.11s | 4.28 / 4.88s |
| 总近停 | 32.01 / 27.00% | 29.04 / 17.57% |
| 最长近停 | 4.20 / 4.59s | 7.59 / 3.66s |
| stale_command近停 | 5.66 / 11.39s | 27.97 / 10.99s |
| 净位移 | 62.07 / 63.48m | 59.65 / 32.01m |
| 采样路程 | 105.35 / 118.49m | 113.81 / 151.53m |

只读改后审计两遍SHA均a96176d1e57e4526fd6117210c0bf09f91962f34d1f0e545bc91227333680d89；各14组请求，<=.15低速等待3.849/3.950s，大部分单组.25–.30s，示例新挡位sent在请求后.319–.558s、实际匹配在同次或后1–2采样可见。该积分与Native near-stop口径不同。平均gear近停减少约81%，换向请求次数未减少，不得称频繁换向根因全部修好。

第一轮stale近停22.01s归于planning；最大5.232s命令间隔为segment_done→route_filter_fallback→连续规划空路径→释放可见接近→GraphNav再规划，4.519s间隔为maneuver_done→footprint_collision→重新规划/航向探针。第二轮净位移明显低而采样路程增加，折返/探索有效性仍须单列。路线和其他GPU负载变化未控制，不能把全部差值归因参数，也不能宣布无回退/稳定性通过。v25只作组件改善的候选，r2保留。

改后>=2s命令间隔7/3次、运动中后台请求30/28次。紧凑文本证据fast_shift_compact.tar.gz（约1.18MB）、fast_shift_results.json、fast_shift_wait_audit.json、fast_shift_runtime_evidence.json已取回；主目录703源文件/备份SHA复核不变。只读审计包装末尾CRLF曾在两遍统计已输出后报错，修正换行后补归档，不混算现场失败。用户随后转向询问目标靠近链路的具体缺口，需基于以上两轮闭包继续查。
