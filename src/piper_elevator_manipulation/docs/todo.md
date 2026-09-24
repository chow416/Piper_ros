# 待办清单

> 截至 2026-09-24。每条都标了**现状证据**（怎么发现的）和**验收标准**（做到什么算完）。
> 优先级：P0 = 直接决定按压能不能用；P1 = 让按压真正闭环；P2 = 工程化；P3 = 能力扩展。

---

## P0 —— 直接决定按压能不能用

### [x] 1. 标定 `press_tcp` 的工具长度（已完成 2026-09-24）

**已改**：`piper_description/urdf/piper_press_tool.xacro` 的 `tool_length` 从占位的
`0.1358` 改为实测值 **`0.1541` m**。xacro 展开已验证（`press_tcp` =
`xyz="0 0 0.1541"`），IK 照常求解、规划成功。

**❗ 这个错误比想象中严重：之前每一次按压都在**物理上超程 18.3 mm**。**

模型的 `press_tcp` 在距安装面 135.8 mm 处，而实物尖端在 154.1 mm 处。软件驱动的是
**模型**的 TCP，所以实物尖端总是比目标**多伸出去 18.3 mm**（工具 +Z 被 IK 对齐到按压方向，
所以这段误差是纯轴向的）。而 TCP 位置本身是用同一个模型做正解算出来的，**软件完全看不到
这个问题**。

叠加当时实测的 -3.5 mm 模型空间偏差，实物尖端实际越过了目标面约 **14.8 mm** ——
而 `press_depth` 只有 3 mm，也就是**约 5 倍的按压行程**。当时压的是空气所以没有后果，
但接真实面板前必须记住这一点。

**验收（还没做）**：跑一次实机按压，确认实物尖端真的停在按钮表面上（不要只看日志，
日志里的 TCP 是模型算的）。建议做法：把尖端抵住已知高度的平台，对比模型 TCP 的高度。

---

### [ ] 1b. 收紧 IK 位置容差（新增 2026-09-24）

工具长度修正后还有一个**模型空间**的残差要查。修正前的实测：

```
worst joint lag: 0.080 deg on joint3          ← 关节滞后只有 0.08°（≈0.4mm @0.3m）
Actual TCP at approach end: (0.2965, 0.0008, 0.2385)
axial offset from target -0.0035 m             ← 模型空间差 3.5mm
```

关节滞后换算过去只有 0.4 mm，**剩下的 3 mm 不是执行误差**，是 IK/规划侧的。

当前容差：`FiveDofIkOptions::position_tolerance = 1.5 mm`、
`CartesianLineOptions::position_tolerance_m = 2.0 mm`。
`five_dof_ik.cpp` 只在 `position_norm <= position_tolerance` 时才返回解，所以每个解本身就允许
偏 1.5 mm。

**做法**：把 `position_tolerance` 收到 `0.3 mm` 重跑，看残差降多少。
注意收敛率会下降（改走 `recovery_ik` 的 32 次随机重启分支会变多），
所以要同时看 `5D IK failed` 的出现频率。

**验收**：`Actual TCP at approach end` 的轴向偏差 < 0.5 mm。

### [x] 2. 按实机标定 `joint_limits.yaml` 的速度/加速度限值（已完成 2026-09-24）

**根因找到了**：厂商 SDK 里写得很清楚（`piper_sdk/piper_msgs/msg_v2/transmit/arm_joint_config.py`、
`MotorMaxSpdSet`）：

| | 单位 | 范围 | 换算 | 之前配的 |
|---|---|---|---|---|
| 关节速度 | 0.001 rad/s | `[0, 3000]` | **≤ 3.0 rad/s** | `5.0` ❌ 超过硬件 |
| 关节加速度 | 0.01 rad/s² | `[0, 500]` | **≤ 5.0 rad/s²** | `1.0` ❌ 只有 1/5 |

再乘上 `approach_speed_scaling: 0.05`，实际加速度只有 **0.05 rad/s² —— 硬件能力的 1/100**。
轨迹永远到不了匀速段，全程都在加/减速 —— **这就是「又慢又抖」的真因**。

**已改**：

- `piper_press_moveit/config/joint_limits.yaml`：`max_velocity 3.0`、`max_acceleration 5.0`
  （厂商硬件上限，不再是偏好值），并写了注释说明「要调快慢就改缩放系数，不要改这里」
- `config/free_motion_route.yaml`：
  `velocity_scaling 0.10→0.20`、`acceleration_scaling 0.10→0.20`、
  `approach 0.05→0.10`、`press 0.01→0.02`、`retreat 0.04→0.08`
- `press_button_demo.cpp`：两处 OMPL 兜底规划器原先**硬编码 0.10**，
  改为读 `free_motion.velocity_scaling` / `free_motion.acceleration_scaling`；
  否则预录路线和采样回退两条分支速度不一致，切换时会突冲。
  新增 `freeMotionScaling()`，与 `free_motion_route.cpp` 的 `readPositiveScaling` 同契约

**实测效果**（新增的计划时长日志，干跑即可读，不用动机械臂）：

| 阶段 | 之前 | 现在 |
|---|---|---|
| via clearance | 2.86 s | 0.90 s |
| from route to pre-press | 4.76 s | 1.29 s |
| advance to button contact | ~5.2 s | 1.63 s |
| press button | 3.16 s | 0.95 s |
| retreat | 4.66 s | 1.34 s |
| return to start | 4.46 s | 1.23 s |
| **合计** | **~25 s** | **7.33 s** |

约 **3.4×** 提速；加速度从 0.05 提到 1.0 rad/s²（**20×**），这是不再抖的主因。

**新增日志**：每次规划后打印各阶段计划时长 + 总计（`planned <stage>  X.XX s`），
实时阶段日志也带上了 `(planned X.XX s)`。

**遗留**：两处硬编码 0.10 已消除，但 `speedScaling()` 仍把 `cartesian.*` 限制在 (0, 0.2]，
而 `freeMotionScaling()` 允许到 1.0 —— 两个上限不一致，见第 14 条（参数集中）。

### [ ] 3. 规划场景加入工作台与面板的碰撞几何

**现状证据**：`press_button_demo.cpp` 里搜不到任何 `CollisionObject` /
`applyPlanningScene`。MoveIt 的规划场景**只有机器人自己**，碰撞检查实际只查自碰撞。
目前臂是「瞎子」，规划出来的路径可能直接穿过桌子或面板。

**做法**（不需要相机）：用卷尺量出桌面、面板、夹具的包围盒，写成
`planning_scene_interface` 的 `CollisionObject` 加进去。可以先用启动参数给出，
后面有标定数据再替换。

**验收**：构造一个「目标点在面板内部」的用例，规划必须**失败**；正常目标仍能规划成功。

### [ ] 4. 急停与故障恢复演练，并把结论写进 `docs/run_press_task.md`

**现状证据**：`hardware_readiness.md` 第 7 条明确写着
「Press timeout, controller fault and emergency stop have not」被演练过。

**要覆盖的场景**：

- [ ] 按压超时（`press_timeout_s`）
- [ ] 控制器掉线 / `controller_failed`
- [ ] 物理急停按下 → 松开 → 如何恢复
- [ ] 按压途中拔掉 CAN

**验收**：四个场景各演练一次，记录「现象 / 恢复步骤 / 是否需要重启 bringup」。

---

## P1 —— 让按压真正闭环

### [ ] 5. 接入接触/力反馈，把「固定行程」换成「接触停止」

**现状证据**：实机走的是 `execution_mode=hardware_fixed_travel`，`press_depth` 就是
停止条件；`/press/contact` 在实机上没有任何发布者。任务书 ③ 只做到了一半。

**候选方案**（按代价排序）：

1. **电机电流**：SDK 有 `GetArmLowSpdInfoMsgs().bus_current`（0.001 A），
   `PiperSystem` 的反馈解析已经覆盖 `0x2A5/0x2A6/0x2A7`，扩到低速度帧即可。
   零额外硬件。
2. 面板侧加微动开关 / 行程开关，走 `std_msgs/Bool`。
3. 末端加力传感器（最贵，但要真力控只能这样）。

**验收**：`require_contact=true` 路径在实机上能跑通；按不动时能报 `CONTACT_NOT_DETECTED`。

### [ ] 6. 接入 TF 变换，放开 `world`/`base_link` 以外的坐标系

**现状证据**：`TargetSource::validate()` 里对未知 frame 直接拒收，日志是
`unknown frame 'camera_color_optical_frame'; no transform path is available yet`。
`hardware_readiness.md` 第 4 条也因此只标了「PARTLY DONE」。

**做法**：`TargetSource` 里加 `tf2_ros::Buffer` + `TransformListener`，把目标从声明
frame 转到 planning frame，同时校验 TF 的时间戳新鲜度（TF 比目标更新才行）。

**验收**：从 `wrist_depth_camera_optical_frame` 或任意已发布 TF 的坐标系发目标，
机械臂能正确按压。

### [ ] 7. 失败重试 + 行为树编排层（BehaviorTree.CPP v4）

**现状证据**：一次失败就退出，必须由上位机重新发目标。目前是「单发」模型，
没有重试、没有恢复策略、没有多步任务。

**⚠️ 版本坑**：Humble apt 里只有 `ros-humble-behaviortree-cpp-v3`（3.8.7，nav2 带进来的），
`BehaviorTree.ROS2` 需要 v4，**必须源码编译**。

**验收**：一次「失败 → 重新规划 → 成功」不再需要人工介入。

### [ ] 8. 连续按压的节流 / 冷却

**现状证据**：`max_presses:=0` 时时长驻节点每次循环 `discardPendingTarget()` 后立刻等新目标。
**没有任何最小间隔**（代码里搜不到 `min_press_interval` 之类）。
如果感知以 5 Hz 持续发布，上一次按完的瞬间就有新目标可用，会连续背靠背按压。

**做法**：加 `min_press_interval_s`（建议默认 2~3 s），两次按压之间强制冷却。

**验收**：`max_presses:=0` + 5 Hz 持续发布时，按压间隔受控，不会背靠背。

---

## P2 —— 工程化

### [x] 9. 让 bringup 自己拒绝重复实例（已完成 2026-09-24）

**现状证据**：2026-09-24 实际踩过两次 —— 两个 `ros2_control_node` 同时写 can0，机械臂收到
互相矛盾的关节指令，表现为「反应很慢、卡住不动」，很容易误判成控制或规划问题。

**已完成**：

1. `scripts/preflight_check.py` 上机前自检（读 `/proc/<pid>/comm` 前缀，
   避开 `pkill -f` 匹配到自己的坑；跳过僵尸进程）。已写进《运行手册》终端 0。
2. `piper_real_bringup.launch.py` 里加了 `OpaqueFunction` 硬防护：
   存在 `ros2_control_node` 时**直接报错退出**，并打印要杀的 PID 和「必须用 `kill -9`」的提醒。
3. `piper_press_moveit.launch.py` 同样加了 `move_group` 防护。（2026-09-24 当天再次踩到：
   一个遗留的 `move_group` 和真实的那个并存，下一次发送目标表现得像根本没收到。）
4. 运行手册「故障排查」补了 `Failed loading controller` 这个症状的成因说明。

**教训**：做实验的脚本（dry-run）一定要把 `move_group` 也一并清掉，
不能只杀 `ros2 launch` 的 PID —— 子进程不一定跟着退。
**新流程**：每次实验结束、以及每次开跑前，都跑 `preflight_check.py`。

### [ ] 10. 版本控制收尾

**现状证据**（`git status --short`）：

- 已跟踪但**有未提交修改**：`press_button_demo.cpp`、`CMakeLists.txt`、`README.md`、
  两个 launch 文件、`package.xml`
- **完全未跟踪**：整个 `config/`、`docs/`、`scripts/`，以及 `target_source.*`、
  `cartesian_line_planner.*`、`free_motion_route.*`、`trajectory_executor.*` 等多个
  头文件和源文件

也就是说**这几天的工作基本都没进版本库**。

**做法**：按主题分几个 commit（硬件接口 / 目标话题接口 / 归位动作 / 文档），不要一次性糊成一个。

**验收**：`git status` 干净；新机器 clone 下来能直接 `colcon build`。

### [ ] 11. 单元测试接线

**现状证据**：`piper_elevator_manipulation` 目录下**没有 `test/` 文件夹**，
`colcon test` 是 `Ran 0 tests`。

**最好测的三个**（全是纯逻辑，不需要硬件）：

- `TargetSource` —— 校验规则（frame / stamp / 四元数 / 工作空间盒）
- `FreeMotionRoute` —— 那 8 条配置校验
- `FiveDofIkSolver` —— 给定构型正解出一个 TCP 位置，再反解回来，检查残差

**验收**：`colcon test` 有真实用例，覆盖上面三块；CI 可跑。

### [ ] 12. 拆分 `press_button_demo.cpp`

**现状证据**：已经约 800 行，`main` / `runSinglePress` / `createTask` /
`executePressSequence` / `JointStateCache` 全挤在一个文件里。

**建议**：拆成 `task_builder`（建 MTC 任务树）/ `task_executor`（执行 + 监控）/
`main`（参数与循环）。`JointStateCache` 可以独立成头文件。

**验收**：单文件不超过 ~300 行，职责单一。

### [ ] 13. 补齐 `ompl_planning.yaml`，明确 planner 配置

**现状证据**：`piper_press_moveit/config/` 里有 `pilz_cartesian_limits.yaml`，
但**没有 `ompl_planning.yaml`**。`press_button_demo.cpp` 里两处
`PipelinePlanner(node, "ompl")` 都没调 `setPlannerId`，实际用哪个 planner、
什么参数不清楚。

自由段的采样回退分支依赖它。现在走的是 `path_source=prerecorded`，
所以问题被掩盖了 —— **一旦预录路线被判定碰撞失效，回退分支的表现是未知的**。

**验收**：明确指定 planner 与超时；人为破坏预录路线（例如加一个障碍），
回退到采样分支能成功。

### [ ] 14. 参数集中管理

**现状证据**：参数散在三处 —— `config/free_motion_route.yaml`（笛卡尔/自由段/限位容差）、
launch 文件里的 `DeclareLaunchArgument`（目标与闸门）、`config/home_pose.yaml`（归位姿态）。

**做法**：至少在每个文件头部写明「还有哪些相关参数在别处」，避免改了 A 忘了 B。
更彻底的做法是收敛成一份主配置 + 少量 launch 覆盖。

**验收**：README 或 `docs/` 里有一张「参数总表」，标明默认值和所在文件。

### [ ] 15. 实机前的轨迹预演工具

**现状证据**：`execute:=false` 只打印日志，实机测试前没有直观手段检查轨迹形状。

**做法**：把规划结果导出成 RViz Marker（或关节角-时间曲线的小图），
让操作员在按下 `execute:=true` 之前能看一眼。

**验收**：能在不动机器臂的情况下看到三个关键位姿和接近方向。

### [ ] 16. 封存感知原型

**现状证据**：`piper_button_perception` 已决定暂停研发，但既没提交进版本库，
也没加 `COLCON_IGNORE`，每次 `colcon build` 仍在编译它。

**验收**：明确它的状态（归档 / COLCON_IGNORE / 移出工作区），并让构建时间下降。

### [ ] 17. 消除限位表与反馈解析的重复

**现状证据**：`piper_press_hardware`(Python) 有 `model_limits.py`、`feedback.py`，
`piper_elevator_manipulation` 和 `piper_hardware` 里也各有一份关节限位表。
同一套协议常量目前散布在至少三处。

**风险**：改一处忘另一处，就会出「两套限位不一致」的隐蔽 bug。

**验收**：关节限位与 CAN 帧解析有唯一来源（例如生成的头文件 + Python 常量）。

---

## P3 —— 能力扩展

### [ ] 18. 多工位点位表

**现状**：一次只能给一个目标点，点位写死在命令行里。
**做法**：把常用点位做成带名字的配置表（例如「一楼外呼」「二楼外呼」「开门」），
目标接口支持按名字指定。

### [ ] 19. 按压过程数据记录
**现状**：只有屏幕日志，事后无法复用。
**做法**：每次按压记录一份 `(t, 关节角, TCP 位姿, 电流)` 的 CSV 或 rosbag，
用于分析 2.2 mm 那类问题、以及做重复精度统计。

---

## 建议的下周顺序

1. **先跑一次实机 Execute**（第 1 条的诊断已经埋好），拿到逐关节报告
2. 根据报告决定：修第 2 条（限值标定）还是做标定补偿
3. 同时并行推进第 3 条（碰撞几何，不需要硬件依赖）
4. 第 10 条（git）**当天就做**，别再拖 —— 现在整个工作区只有一份工作副本
