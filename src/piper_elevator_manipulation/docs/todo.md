# 待办清单

> 更新 **2026-10-14**。每条都带**现状证据**（怎么发现的）和**验收标准**（做到什么算完）。
>
> 优先级：**P0 = 当前主线** → P1 = 真机能不能用 → P2 = 让按压闭环 → P3 = 工程化 → P4 = 能力扩展。
> 末尾「明确不做」收录已经拍板**不在计划内**的事，避免反复拿出来讨论。
>
> 条目编号自 2026-09-24 起**不重排**，新条目从 20 往后加 —— 正文里有「见第 14 条」
> 这类交叉引用，重排会让它们全部指错。

## ⚠️ 两条硬约束（用户拍板，不要试图绕过）

1. **启动流程一个字都不变**（2026-10-14 原话：「这是之前单个标定板进行按压的终端
   启动流程，现在两个标定板，启动流程也别变。保持这样。」）。终端数、命令、交互
   全部保持单板时代的样子；双板支持只能塞进 `press_board.sh --serve` 内部。
   任何「再开一个终端跑 XXX」的方案**不合格**。
2. **不做接触检测**。硬件上没有任何接触手段，按压段按固定行程判定。见「明确不做」。

---

## P0 —— 当前主线：视觉二次修正

设计文档在**外层仓库**：`docs/superpowers/specs/2026-10-12-visual-refinement-design.md`。
一句话：**远处 3×3 大板粗定位 + 按键旁单码精修正，位置融合、法向只用第一次观测**。

### [ ] 20. `tip_in_board` 实机验收（A 板）★ 最先做

**现状证据**：2026-10-14 实测量得**按键在大板的左上**，`board.yaml` 的 `tip_in_board`
已从占位的 `{0.045, 0.238}` 换成实测的 **`{−0.175, +0.160, 0}`**，`taught: true`。
⚠️ **但这只是把尺子量出来的数填进去，没有实机按过。**

⚠️ 量完必须抬头确认「面向板子看，按键在板心的**右上还是左上**」—— 板系 `+X` 是
**印在纸上的方向**，不是世界的方向。板子转着贴、贴反，符号就整体翻，而**代码和
四元数都看不出来**，只会把杆送到镜像位置去。这次实测就是「横向 17.5 cm 但在左上」，
所以是 **−0.175**，不是正。

**做法**：`tools/demo.sh --button call`（一键脚本，一条命令跑完原终端 3 + 终端 4）。
先加 `--dry` 干跑，确认能解出 IK、规划成功，再实机跑一次。

**干跑顺带看**：`marker_locator` 在 `go_home` 位姿下能不能正常读出 A 板。

**验收**：杆尖落在**按键中心**（目视几毫米内），不是板心。

### [ ] 21. B 码（ID 9）在 `pre_press` 位姿下可见吗 ★ 决定方案成不成立

**现状证据**：spec §7 未决数据第 1 条，**从来没测过**。第二段精修正整个建立在
「臂停在 `pre_press` 时，腕部相机能看到按键上方那个 28.5 mm 单码」之上。

⚠️ 这个风险是**非对称**的：如果看不到，第二段没有替代方案，整个「两次观测」设计
要推倒。所以**必须在写任何代码之前测**，否则可能写完一整套才发现地基是空的。

**做法**：让臂停在 `pre_press`（干跑算出的位姿），看相机；或实机按压途中抓一帧。

**验收**：`min_markers=1` 下能从 `pre_press` 解出 ID 9 的位姿，抖动可接受。

**已知量化背景**：单码噪声底比 3×3 高得多（3×3 实测抖动 0.3~1.3 mm，单个
28.5 mm 码 2.3~3.3 mm）。`demo.sh --max-jitter-mm` 的默认值是按 3×3 定的，
单码会被误报成「板子在动」，要放宽。

### [ ] 22. 修正 `run_press_task.md` 里的失能描述（安全相关，10 分钟）

**现状证据**：2026-10-14 实测 + 日志核实 —— **任何「停进程」的方式都不会让臂失能**。
使能/失能是**电机固件**的状态，只由一帧 CAN `0x471` 改变；全仓库只有
`PiperSystem::on_deactivate()`（`piper_system.cpp:421`）会发它，而它**从来没被调用过**
（Ctrl-C 走的是硬件的 `on_shutdown()`，`PiperSystem` 没重写它）。

现有文档里两处**与事实相反**：

- `run_press_task.md:327`「（控制器管理器退出会失能，臂会落到硬限位上）」
- `run_press_task.md:362`「`kill -9 <pid>  # 用 -9！普通 kill 会走失能钩子，臂会掉到硬限位」

第二条尤其危险：它把「用 `-9`」说成保命操作，而**普通 `kill` 和 `kill -9` 毫无区别**。

**验收**：文档与实测一致，并写清**唯一**能真正失能的办法
（`ros2 control set_hardware_component_state PiperSystem inactive`）。

### [ ] 23. `marker_locator` 支持双板

**现状证据**：现在是**单板节点** —— 一个 `self.spec`、一次 `detect()`、一个话题。
PnP 层本身没问题（`aruco_board.py` 的对象点表就是白名单，不在表里的 marker 进
`solvePnP` 前就被 `continue` 掉），**风险全在节点层，而且是静默的**。

**四条必守项**（spec §3.2，每一条都能单独把事情搞坏）：

1. 两块板**各解一次、各发各的话题** —— 写成 `coarse or fine`（谁先解出就发谁）
   会把 A 板位置发到精修正话题上，而日志 / RMS / marker 数**全部正常**
2. `accepted_ids` **互斥 + 加载时硬校验** —— 整个隔离完全建立在不重叠上
3. **每块板各存一份几何** —— 现在 `board.yaml` 只有一份 `marker_length`/`separation`，
   而打印缩放是**系统性**的
4. **`min_markers` 是主判据，RMS 不是** —— 板子贴反时 9 码只剩 1 码，那 1 码能让
   `solvePnP` 拟合出 **RMS 0.14 px** 的漂亮错位姿

**验收**：两块板同框时各发各的话题，A 板位置不会出现在精修正话题上。

### [ ] 24. `press_target_bridge.py`：复活或删除，不要两边都留

**现状证据**：它**已安装、已在 `CMakeLists.txt`、已写进 `README.md`、还有集成测试**
（`test_bridge_live.py`），但 `press_board.sh` **从来不起它** —— 实际链路走的是
`tools/press_board_centre.py`（一次性 CLI，`press_board.sh:464` 每按一次跑一次）。
**文档里有、链路里没有**，这种状态最容易误导人（本次就误导过我一次）。

**为什么它现在又被需要**：精修正必须在 `pre_press` 那一刻**在线**工作，一次性 CLI
做不到 —— 而 bridge 是常驻节点，**正好是需要的形态**。

**验收**：要么它在 `press_board.sh --serve` 里真的被起起来，要么它和它的 README
章节一起删掉。**没有第三种状态。**

### [ ] 25. 拆 `executePressSequence` → `runToPrePress` + `executeFromPrePress`

**现状证据**：spec §3.5。精修正的断点就在这里 —— 必须能在 `pre_press` 停住、
拿一次观测、再继续。

**验收**：`max_refinement_rounds=0` 时行为与现在**逐位一致**（先立回归基线，再改）。

### [ ] 26. 排查 CAN `No buffer space available`

**现状证据**：2026-10-14 的 `ros2_control_node` 日志里有

```
[ERROR] [PiperSystem]: CAN write failed: write(CAN): No buffer space available
```

（`RCLCPP_ERROR_THROTTLE`，2 s 一次）。**同一天** `go_home.py` 报了
`error_code = 0` / `Goal reached, success!`，而**关节一度一动不动**（前后逐关节零差异）。
时间上挨得很近。

**存疑**：不是规划/控制问题，而是**命令压根没发上 CAN**。注意同日日志里还有
**两个** `ros2_control_node` 在 23 秒内先后 `configure` 了 `PiperSystem`，
字节数最小的那个 3 秒后就没了 —— 重复实例的嫌疑不能排除（见第 9 条）。

**验收**：能复现时定位到根因；或证明只是瞬态。

---

## P1 —— 真机能不能用

> 第 1、2 条已完成，正文留作证据。未完成的是 **1b、3、4**。

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

## P2 —— 让按压真正闭环

### [~] 5. ~~接入接触/力反馈~~ —— 已拍板不做，见「明确不做」

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

## P3 —— 工程化

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

### [x] 10. 版本控制收尾（本地已完成 2026-09-24）

**2026-09-24 已完成本地提交**，在 `piper-press-dev` 分支上分五个主题提交：
`piper_hardware` 新包 / `piper_press_hardware` 新包 / 工具长度与关节限值标定 /
`move_group` 重复实例防护 / `piper_elevator_manipulation` 按压任务与接口。
合计 66 个文件、8532 行新增，`git status` 干净。

其中 `piper_press_hardware` 是当时的调试包，现已从工作区移除；
此处仅保留原提交记录。

**✅ 2026-10-13 更新：远程备份早已做完，上面那一节原本的内容（「没有远程备份」
「只存在于这一台机器上」）已全部作废。**

现在是**三个**远端，而且 **推 GitHub 完全不更新 GitLab**（那份是扁平快照，没有 git 血缘）：

| 仓库 | 远端 | 分支 |
|---|---|---|
| `src/Piper_ros`（内层） | GitHub `chow416/Piper_ros` | **`piper-press-dev`**（⚠️ 内层**没有 `main`**）|
| `/home/zlf/piper_ws`（外层） | GitHub `chow416/piper_ws` | `main` |
| 公司 GitLab `elevator-arm-flow-verify` | `code.pudu.work`（**内网**）| `initial-import`（`main` 受保护）|

- 官方仓库已改名 `upstream` 保留只读参考。
- ⚠️ **用户约定：「推远端」= 同时推 GitLab + GitHub。** GitLab 那份每次都要重新
  rsync（快照在 `~/eafv-snapshot`）。排除清单里 **`/docs/` 的斜杠必须打头** ——
  写成未锚定的 `docs/` 会**静默**排掉所有层的 docs（2026-10-13 因此少了 57 个文件）。
- `src/third_party/`：`moveit_task_constructor` 和 `py_binding_tools` **都是正式子模块**，
  新机器 `clone --recurse-submodules` 能编译；**只有 `piper_sdk` 不在版本控制里**
  （它有 `COLCON_IGNORE`，`piper_hardware` 也不依赖它，不影响编译）。

### [ ] 11. 单元测试接线

**现状证据**：`piper_elevator_manipulation` 目录下**没有 `test/` 文件夹**，
`colcon test` 是 `Ran 0 tests`。

**最好测的三个**（全是纯逻辑，不需要硬件）：

- `TargetSource` —— 校验规则（frame / stamp / 四元数 / 工作空间盒）
- `FreeMotionRoute` —— 那 8 条配置校验
- `FiveDofIkSolver` —— 给定构型正解出一个 TCP 位置，再反解回来，检查残差

**验收**：`colcon test` 有真实用例，覆盖上面三块；CI 可跑。

### [x] 12. 拆分 `press_button_demo.cpp`（已完成 2026-10-12）

**结果**：**1288 → 238 行**，新增 7 个模块（`.hpp` + `.cpp`）：
`press_constants` / `press_solution` / `joint_state_cache` / `press_parameters` /
`press_task_builder` / `press_executor` / `press_runner`。

**三层验收都过**：① 全量重建零错误零警告 ② `scripts/verify_pure_move.py --before 067d404`
→ **27 个符号逐字一致** ③ **实机完整按压一趟**（六个相位全执行，关节跟踪最差
0.079°（接近段）/ 0.114°（按压段），模型空间误差 0.2 mm，回起点差 ≤ 0.07°）。

⚠️ 这只是让过渡形态**可维护**，**没有**把它变成行为树 action server ——
契约里「已知过渡缺陷」的定性仍然成立（见第 7 条）。

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

### [x] 16. 封存感知原型（已完成 2026-09-28）

**做法**：整体移除，不是归档。

- 删除 `piper_button_perception` 包（19 个文件）及其 `build` / `install` 残留。
- 删除训练与感知容器配置：`docker/Dockerfile.training`、
  `docker/docker-compose.training.yml`、`docker/docker-compose.perception.yml`、
  `docker/requirements-training.txt`。
- 删除归档设计文档 `docs/button_perception_plan.md`。
- 数据集与训练权重共 68 MB 移到工作区外
  `~/piper_perception_archive_20260928/` 备份，**未进版本库**；这两个目录原本
  就在 `.gitignore` 里，删掉不可用 git 恢复。

**结果**：`colcon build` 不再编译该包。后续感知改为识别标定板，不再依赖
神经网络；标定结果写回 `wrist_depth_camera.xacro` 的 fixed joint origin，
使 TF 成为唯一真相，感知层与规划层都不需要知道标定细节。

### [ ] 17. 消除限位表与反馈解析的重复

**现状证据**：旧的 `piper_press_hardware` 已移除，但
`piper_elevator_manipulation` 和 `piper_hardware` 里仍各有一份关节限位表。
同一套协议常量目前仍散布在至少两处。

**风险**：改一处忘另一处，就会出「两套限位不一致」的隐蔽 bug。

**验收**：关节限位与 CAN 帧解析有唯一来源（例如生成的头文件 + Python 常量）。

---

## P4 —— 能力扩展

### [ ] 18. 多工位点位表

**现状**：一次只能给一个目标点，点位写死在命令行里。
**做法**：把常用点位做成带名字的配置表（例如「一楼外呼」「二楼外呼」「开门」），
目标接口支持按名字指定。

### [ ] 19. 按压过程数据记录
**现状**：只有屏幕日志，事后无法复用。
**做法**：每次按压记录一份 `(t, 关节角, TCP 位姿, 电流)` 的 CSV 或 rosbag，
用于分析 2.2 mm 那类问题、以及做重复精度统计。

---

## 明确不做（已拍板，别反复拿出来）

### [~] 5. 接触检测 / 力反馈

用户 2026-10-13 原话：

> 目前做不到接触检测，没有任何接触检测的手段，按压段只能按照固定按压行程的方式
> 来判定是否成功。我之前没说过要做接触检测，就算要做也不是目前的任务。

**硬件事实**：**没有**任何接触传感器；`/press/contact` 在实机上**没有发布者**；
`execution_mode=hardware_fixed_travel` 是唯一能走的路（轨迹即停止条件，
`hardware_travel_margin_m` 之上的实测 TCP 行程作为独立安全上限）。

⚠️ 别把它写成「现状不在本设计范围内」—— 那样读起来像**默认存在**接触检测。
要如实写成**硬件现实**。

⚠️ 但架构契约（`.cursor/skills/piper-press-planning-control/ARCHITECTURE.md` §8）
把「补真实接触传感器」排在行为树 action server **之前**。这个顺序冲突**当前无解**，
以用户拍板为准 —— 第 7 条（行为树）因此也一并搁置。

---

## 下一步顺序（2026-10-14 重排）

1. **第 20 条** —— `tip_in_board` 实机验收（`tools/demo.sh --button call`，先 `--dry`）
2. **第 21 条** —— B 码可见性。**它决定第二段方案成不成立，必须在写代码之前做**
3. **第 22 条** —— 修 `run_press_task.md` 的两处错误说法（10 分钟，安全相关）
4. 以上都过了，才动 23 → 24 → 25（双板感知 → 桥接复活 → 断点拆分）
5. 第 3 条（碰撞几何）**不需要硬件**，可以全程并行

> 旧的「下周顺序」（先跑实机 Execute → 修限值 → git 当天做）已经全部完成，作废。
