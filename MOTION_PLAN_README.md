# fast 路径感知运控参数说明

本文档覆盖 `fast` 分支新增路径规划器、计划执行状态机，以及与它直接相关的现有底盘参数。默认参数以 0.2 m 网格、10 ms 运控周期为前提。

## 修改位置

- 路径级参数：`src/motion_plan.c` 中的 `motion_plan_default_config`。
- 在线控制参数：`src/move_control.c` 中以 `MOTION_` 开头的常量。
- 底盘内环参数：`src/move_control.c` 顶部和 `move_control_init()`。
- 编码器换算参数：`inc/move_control.h` 中的 `SPEED_COEFFICIENT`。

主流程当前把 `motion_plan_default_config` 同时作为求解器路径规划参数。在线加减速度限制也读取该默认配置，因此修改加减速度时应直接修改默认配置，不要只在局部创建另一份临时配置。

## 路径规划参数

| 参数 | 默认值 | 单位 | 作用 | 调大后的主要影响 |
| --- | ---: | --- | --- | --- |
| `max_cruise_speed_mps` | 0.60 | m/s | 所有线段的最高巡航速度 | 长直线更快，制动距离和横偏风险增大 |
| `segment_base_speed_mps` | 0.15 | m/s | 短线段限速公式的基础速度 | 所有短段整体提速 |
| `segment_length_gain` | 0.75 | 1/s | 线段长度对限速的增益 | 速度随线段长度增长得更快 |
| `corner_speed_mps` | 0.08 | m/s | 普通 90 度转角的通过速度 | 停顿更少，但转角外甩和横偏增大 |
| `max_accel_mps2` | 1.00 | m/s^2 | 前向速度规划和在线矢量加速度上限 | 起步和换向更快，轮胎打滑及电流冲击增大 |
| `max_decel_mps2` | 1.20 | m/s^2 | 后向速度规划和在线减速度上限 | 更晚制动，停稳冲击增大 |
| `vision_distance_m` | 0.80 | m | 两次风险锚点视觉校正间的累计路程 | 校正次数减少，累计里程计误差增大 |
| `vision_turn_count` | 4 | 个 | 两次风险锚点视觉校正间的普通转角数 | 校正次数减少，连续折线累计误差增大 |
| `explosion_dwell_ms` | 1310 | ms | 爆破节点停车等待时间 | 爆破更稳妥，但总耗时增加 |

线段限速公式为：

```text
segment_limit = min(max_cruise_speed_mps,
                    segment_base_speed_mps + segment_length_gain * length_m)
```

默认参数下，0.2 m 一格短段限速为 0.30 m/s。风险锚点满足“累计路程达到阈值”或“累计转角达到阈值”任一条件即触发。

## 路径节点精简规则

- 连续重复坐标合并，动作标志和最长等待时间合并到同一节点。
- 无动作的同向共线中间点删除。
- 只有 `MOTION_STOP | MOTION_MACRO_END` 的普通宏动作节点，如果与前后路径同向共线，也会删除以避免直线中停车。
- 90 度转角、180 度掉头、终点、视觉校正节点和爆破等待节点始终保留。
- 斜线、连续错误 `255`、未知动作位和容量溢出会让规划失败，主流程保持停车。

## 在线控制参数

| 参数 | 默认值 | 单位 | 作用 | 调参注意事项 |
| --- | ---: | --- | --- | --- |
| `MOTION_PASS_PLANE_M` | 0.002 | m | 普通转角切段的投影平面窗口 | 调大可更早切段，但节点位置误差增大 |
| `MOTION_POSITION_TOLERANCE_M` | 0.015 | m | 节点位置容差和横偏安全阈值 | 比赛目标要求最大横偏不超过 1.5 cm，不应调大 |
| `MOTION_START_ANCHOR_TOLERANCE_M` | 0.05 | m | 是否跳过首锚点对齐 | 调大可更快起步，但初始路径偏移可能更大 |
| `MOTION_STOP_SPEED_MPS` | 0.03 | m/s | 停稳判定的平移速度阈值 | 调大更快判停，但动作节点可能仍在滑动 |
| `MOTION_STOP_STABLE_TICKS` | 3 | 10 ms | 停稳速度需要连续满足的周期数 | 调大更稳，但每个停车点耗时增加 |
| `MOTION_STOP_TIMEOUT_TICKS` | 50 | 10 ms | 停车失败的安全中止时间 | 默认 500 ms；过大可能掩盖控制故障 |
| `MOTION_VISION_TIMEOUT_TICKS` | 50 | 10 ms | 节点视觉等待或采样超时 | 默认 500 ms；调大增加阻塞时间 |
| `MOTION_CROSS_TRACK_GAIN` | 2.0 | 1/s | 横向误差到修正速度的比例增益 | 回线更快，但可能左右振荡 |
| `MOTION_CROSS_CORRECTION_LIMIT_MPS` | 0.10 | m/s | 正常跟踪时横向修正速度限幅 | 大偏差恢复更快，但会占用更多矢量加速度 |
| `MOTION_RECOVERY_SPEED_MPS` | 0.10 | m/s | 横偏或节点误差后的最高恢复速度 | 恢复更快，但重新对点更容易过冲 |
| `MOTION_START_RECOVERY_SPEED_MPS` | 0.15 | m/s | 首锚点对齐最高速度 | 起步对齐更快，但初始位姿误差大时风险增加 |
| `MOTION_NODE_SPEED_TOLERANCE_MPS` | 0.005 | m/s | 普通转角切段时允许的节点速度误差 | 调大可更早切段，但转角外甩增大 |
| `MOTION_RECOVERY_POSITION_GAIN` | 2.0 | 1/s | 恢复对点距离比例增益 | 靠近目标前速度更高，过冲风险增大 |
| `MOTION_RECOVERY_MIN_SPEED_MPS` | 0.02 | m/s | 恢复对点最低指令速度 | 可克服静摩擦，但过大时难以进入 2 mm 范围 |
| `MOTION_VISION_STABLE_NORMALIZED_TOLERANCE` | 0.002 | 归一化坐标 | 节点视觉相邻帧稳定阈值 | 调大更易得到三帧稳定，视觉抖动误差也增大 |
| `MOTION_VISION_STABLE_FRAMES` | 3 | 帧 | 节点视觉稳定帧数 | 调大更稳，但视觉停车时间增加 |

恢复对点速度为：

```text
recovery_speed = clamp(MOTION_RECOVERY_POSITION_GAIN * distance
                       + MOTION_RECOVERY_MIN_SPEED_MPS,
                       MOTION_RECOVERY_MIN_SPEED_MPS,
                       recovery_speed_limit)
```

普通转角必须同时满足以下条件才无停车切换下一段：进入 `MOTION_PASS_PLANE_M` 窗口、横偏不超过 `MOTION_POSITION_TOLERANCE_M`、沿线速度不高于节点速度加 `MOTION_NODE_SPEED_TOLERANCE_MPS`。

## 结构与时间参数

以下参数只有在硬件、地图或调度周期变化时才能联动修改，不属于日常实车调参。

| 参数 | 默认值 | 位置 | 必须同步检查的内容 |
| --- | ---: | --- | --- |
| `MOTION_CONTROL_DT_S` | 0.01 s | `src/move_control.c` | PIT 周期、超时 tick、加速度步长、主机仿真周期 |
| `GRID_SIZE_M` | 0.2 m | `src/motion_plan.c` | `motion_grid_to_world()` 中的 0.2/0.1、地图识别坐标 |
| `MM_PER_M` | 1000 | `src/motion_plan.c` | `MotionNode` 的 mm/s 单位约定，通常不修改 |
| `WIDTH`、`MAP_SIZE` | 16、192 | `algorithm_inc/sokoban_engine.h` | 地图高度、格索引转换、世界坐标范围 |
| 世界坐标宽高 | 3.2 m、2.4 m | `motion_grid_to_world()` 和节点视觉换算 | OpenART 归一化坐标、地图尺寸 |

`run_vision_correct()` 的 0.05 s 触发周期、0.1 s 通信超时、横移修正 `global_y`、纵移修正 `global_x`，以及 PIT 中 `move_control_task(); run_vision_correct();` 的顺序是本方案硬约束，不应作为 fast 运控参数修改。

## 现有底盘参数

这些参数本次没有改动，但会直接影响新路径计划能否真实跟随。除非已经确认外环参数合理，否则不要与路径参数同时修改。

| 参数 | 默认值 | 作用 |
| --- | ---: | --- |
| 四轮速度 PID `Kp/Ki/Kd` | 40.0 / 2.0 / 0.0 | `move_control_init()` 中的轮速内环 |
| `Kp_yaw` / `Kd_yaw` | 0.06 / 0.10 | 航向 PD |
| 航向输出限幅 | +/-0.60 | `yaw_pid_calculate()` 的旋转速度限幅 |
| 航向到达阈值 | +/-1.0 度 | 小于该误差时停止航向修正 |
| `max_yaw_step` | 10 度/周期 | 瞬时目标航向的变化上限 |
| `vx_encoder_index` | 0.925 | 横向里程计比例修正 |
| `vy_encoder_index` | 1.0 | 纵向里程计比例修正 |
| `SPEED_COEFFICIENT` | 由 1024 线、10 ms、3/7 减速比、31.5 mm 轮半径计算 | 编码器计数到 m/s 的换算 |

旧 `car_move()` / `car_move_point()` 还使用 `min_distance=0.01 m`、`kp_position_x/y=4.0`、`kd_position_x/y=0.0` 和 `max_speed=1.0 m/s`。这些参数只影响旧 A/B 回退及点到点动作，不影响 `car_move_plan()` 的线段前馈控制。

## 推荐调参顺序

1. 固定地图、电池电压和轮胎状态，先校准 `SPEED_COEFFICIENT`、`vx_encoder_index`、`vy_encoder_index`。
2. 用低速直线确认四轮速度 PID 和航向 PD 稳定，不振荡、不持续饱和。
3. 只调整 `max_accel_mps2`、`max_decel_mps2` 和 `max_cruise_speed_mps`，确认长直线能够稳定停车。
4. 用至少六段 0.2/0.4 m 短折线调整 `corner_speed_mps`，保持最大横偏不超过 0.015 m。
5. 若正常轨迹存在缓慢横偏，先调 `MOTION_CROSS_TRACK_GAIN`，再调修正速度限幅。
6. 最后调整风险锚点间隔和视觉稳定参数，避免用更频繁停车掩盖里程计或内环问题。

每次只改变一组参数，至少连续运行 10 次，记录总耗时中位数、最大横偏、停车次数、恢复次数、视觉超时次数和安全中止次数。新方案验收目标仍为：无漏动作、无安全中止、最大横偏不超过 1.5 cm，总耗时中位数不高于旧方案的 80%。
