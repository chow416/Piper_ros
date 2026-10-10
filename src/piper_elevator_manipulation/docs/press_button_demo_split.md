# press_button_demo.cpp 功能切分 — 设计规格

**状态**：待评审（未动工）
**日期**：2026-10-10
**上游契约**：`.cursor/skills/piper-press-planning-control/ARCHITECTURE.md`
（§6 "Package boundaries → Target evolution"、§7 "Readability and maintenance standard"）

---

## 1. 目标与成功标准

**目标.** 把 1288 行、21 个顶层定义的 `press_button_demo.cpp` 拆成
「**一个模块一个变更理由**」的文件集合，让新开发者能单独读懂每一块。

**成功标准（全部可验证）.**

| # | 判据 |
|---|---|
| 1 | `colcon build --symlink-install --packages-select piper_elevator_manipulation` 通过 |
| 2 | **同一条 dry-run 命令的终端输出逐字一致**（见 §5.2） |
| 3 | 可执行文件仍然只有一个：`press_button_demo` |
| 4 | 对外零接口变化 —— 其他包、launch、`tools/*.sh` 一行都不用改 |

**非目标（本次明确不做）.**

- 不改任何行为、默认值、日志文本
- 不引入新库，不改 CMake 的目标结构（仍是单个 `add_executable`）
- 不修 §6 列出的 4 个已知缺陷
- 不动 `free_motion_route.cpp` / `cartesian_line_planner.cpp` / `five_dof_ik.cpp`
  等已经分好的模块

---

## 2. 现状（1288 行的真实构成）

```
 46 ┌ namespace {  ………………………………………… 第一个匿名 namespace ── 742 行
 47 │   kArmGroup / kEndEffectorLink / kPlanningFrame / kArmJoints
 53 │   struct PressTaskParameters
 79 │   struct PlanningTask
 86 │   struct PlannedMotion
 91 │   collectMotions
116 │   motionTimeout
124 │   solutionUsesStage
142 │   template getOrDeclareParameter
158 │   speedScaling
172 │   freeMotionScaling
185 │   plannedSeconds
192 │   vector3Parameter
204 │   pressTimeout
217 │   class JointStateCache
294 │   kRadiansToDegrees
305 │   lastCommandedPositions
320 │   logJointTracking
352 │   waitForSettledJoints
386 │   solveWaypoint
402 │   createTask
565 │   createRecoveryTask
614 │   executePressSequence
788 └ }
790 ┌ namespace {  ………………………………………… 第二个匿名 namespace ── 288 行
810 │   struct EditablePressParams
820 │   readPressParamsFile
863 │   struct RunSettings
872 │   validatePressParameters
914 │   waitForTarget
957 │   runSinglePress
1078└ }
1080  main  ………………………………………………………………………………… 209 行
```

**两个匿名 namespace 的分界线（788/790）没有概念依据** —— 它是历史演进的沉积，
不是设计。这是本次要消除的东西之一。

---

## 3. 目标结构

七个新模块 + 瘦身后的 `main`。**全部编译进同一个可执行文件**，不建库。

| 新文件 | 职责（一句话） | 从原文件搬来的东西 | 估计行数 |
|---|---|---|---|
| `press_constants.hpp` | 跨模块共享的字面量 | `kArmGroup` `kEndEffectorLink` `kPlanningFrame` `kArmJoints` + 4 个 stage 名 | ~35 |
| `press_solution.{hpp,cpp}` | 把 MTC 解树变成可执行/可打印的东西 | `PlannedMotion` `collectMotions` `solutionUsesStage` `plannedSeconds` `motionTimeout` | ~110 |
| `joint_state_cache.{hpp,cpp}` | 读实测关节角，并判断"到没到、跟没跟上" | `JointStateCache` `kRadiansToDegrees` `lastCommandedPositions` `logJointTracking` `waitForSettledJoints` | ~185 |
| `press_parameters.{hpp,cpp}` | 按压请求的类型、获取、校验 | `PressTaskParameters` `EditablePressParams` `readPressParamsFile` `validatePressParameters` `getOrDeclareParameter` `speedScaling` `freeMotionScaling` `vector3Parameter` `pressTimeout` | ~190 |
| `press_task_builder.{hpp,cpp}` | 把请求变成一棵**未规划**的 MTC 任务树 | `PlanningTask` `solveWaypoint` `createTask` `createRecoveryTask` | ~240 |
| `press_executor.{hpp,cpp}` | 把已规划的解按相序执行下去 | `executePressSequence` | ~180 |
| `press_runner.{hpp,cpp}` | 一次完整流程的编排（建→算→跑） | `RunSettings` `waitForTarget` `runSinglePress` | ~180 |
| `press_button_demo.cpp` | 进程入口 + 主循环 | `main` | ~210 |

**依赖方向（无环）：**

```
press_constants.hpp   ← 叶子，只含字面量
        ↑
press_parameters      press_solution      joint_state_cache
        ↑                    ↑                    ↑
        └──── press_task_builder ────┐              │
                                     ↓              │
                              press_executor ←──────┘
                                     ↑
                              press_runner
                                     ↑
                          press_button_demo.cpp (main)
```

**头文件位置**：`include/piper_elevator_manipulation/`（与既有 7 个模块一致）。
**实现位置**：`src/`。**CMake**：加进 `add_executable(press_button_demo ...)` 的源列表。

---

## 4. 边界决策（为什么这么切）

| 决策 | 理由 |
|---|---|
| `PressTaskParameters` 放 `press_parameters` 而不是 `press_task_builder` | 它是**请求契约**（契约 §4 "Stable data contracts"），且被 builder / executor / runner 三方消费。放参数模块 = 类型和它的校验/读取在一起 |
| `motionTimeout` 跟 `plannedSeconds` 走，`pressTimeout` 跟参数走 | 前者输入是 `JointTrajectory`（轨迹问题），后者输入是 node（参数问题） |
| `waitForTarget` 放 `press_runner` | 它只被 `main` 调用，和 `runSinglePress` 同属编排层 |
| **不建库**，仍是一个 `add_executable` | 建库会引出 install/export 的问题，而目前没有任何第二方消费这些符号 —— YAGNI |
| 保留 `getOrDeclareParameter` 等通用 ROS 助手的现有实现 | 只搬家，不改写 |

---

## 5. 验证方法

### 5.1 编译

```bash
# 容器内
cd /ws && colcon build --symlink-install --packages-select piper_elevator_manipulation
```

### 5.2 干跑输出逐字对比（核心判据）

**改之前先取基线**，改完用**同一条命令**再跑，`diff` 两份输出。

必查的几行（少一行、变一个数字都算失败）：

```
Press parameters (from ...): approach ... mm, contact standoff ... mm, press depth ... mm
Free motion route 'tabletop_staging' has N prerecorded waypoint(s).
5D IK solved from seed '...'.
free_motion.path_source=prerecorded
  planned <每一段>                        X.XX s
  planned total (motion only)             X.XX s
Planning succeeded: approach, contact, press, retreat, return.
```

⚠️ 命令**以 `execute:=false` + 电机不使能为前提**，不移动机械臂。
具体 launch 参数在动手前定下来并记进基线文件。

### 5.3 静态检查

- `grep` 确认 `press_button_demo.cpp` 里不再有 `createTask` / `executePressSequence` 等定义
- `nm`/`ldd` 确认产物仍只有一个可执行文件

---

## 6. 已决议：stage 名**不**提常量（2026-10-10）

> ✅ **决定：保持裸字符串，本次严格只搬文件。** 归到后续的 bug 修复里一起处理。
> ⇒ `press_constants.hpp` 只放原有的 4 个常量（`kArmGroup` / `kEndEffectorLink` /
> `kPlanningFrame` / `kArmJoints`），**不加 stage 名**，行数估计降到 ~20。
> ⚠️ 代价：切分后 stage 名的裸字符串会**分散在 3 个模块里**，是本设计已知的遗留
> 脆弱点，记录在案。

以下为决议时的背景分析（保留备查）。

**把 4 个 stage 名提成 `constexpr` 常量。**

```
"advance to button contact"  "press button"
"retreat to pre-press waypoint"  "return to captured start state"
```

它们在原文件里以**裸字符串**出现 **14 次**（定义处 5 次、查询处 5 次、
恢复任务 2 次、校验 2 次）。切分后它们会跨 3 个模块 —— 再靠裸字符串互相对齐
就是切分**新引入**的脆弱点。

**严格说这不是"纯搬家"**：代码变了（虽然行为不变，§5.2 能验证）。
所以标出来等你决定：

- **要做**：`press_constants.hpp` 里定义，三处引用改成常量。风险接近零
- **不做**：保持裸字符串，本次严格只搬文件

---

## 7. 明确排除的 4 个缺陷（本次不修）

| # | 位置 | 问题 |
|---|---|---|
| 1 | `validatePressParameters` | 真正的下限是 `CartesianLineOptions::position_tolerance_m`（2 mm，**硬编码**），不是 `cartesian.max_step_m`（2 mm，**可配**）。两数碰巧相等，规则"能用但理由错"，两个方向都判错 |
| 2 | `executePressSequence`（原 642 行） | `unexpected phase order` 直接 throw，`catch` 里没有撤退 → **机械臂留在按钮上** |
| 3 | 同上 | 错误信息不打印**实际收到的阶段名列表** |
| 4 | 启动校验 | 缺 `cartesian.max_step_m > position_tolerance_m` 的检查。不满足时**每个采样点的 IK 都原地不动**，静默缩短每段行程 |

**顺序建议**：切分完成后，按 1→4→2→3 单独修（2 和 3 强相关，放一起）。
先切后修的理由：切完每个缺陷都落在**职责明确的单文件**里，改动半径可预测。

---

## 8. 风险

| 风险 | 缓解 |
|---|---|
| 搬家过程中漏掉一处 `static`/匿名 namespace 的可见性变化 | 全部新符号放具名 namespace `piper_elevator_manipulation`；§5.1 编译即暴露 |
| 搬到新文件后忘记 `#include`，靠传递包含侥幸编译通过 | 编译通过 + §5.2 跑通 |
| `install/` 是 `--symlink-install`，新文件不会自动进 install | `colcon build` 会重新生成 `install/`（不是纯符号链接时才需要） |
| 改动落地后又发现行为变了，回头看 baseline 已过期 | §5.2 要求**先取基线再动手**，基线文件随改动一起提交 |
