# press_button_demo.cpp 功能切分 — 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 1288 行的 `press_button_demo.cpp` 按职责拆成 7 个模块 + 瘦身 main，
**零行为变化**（判据：同一条干跑命令的输出逐字一致）。

**Architecture:** 全部新文件仍编译进**同一个可执行文件** `press_button_demo`，不建库。
每个模块的内容包在 `namespace piper_elevator_manipulation { ... }` 里，
这样共享常量可以不带限定符直接使用（与包内既有 7 个模块一致）。

**Tech Stack:** C++17、ROS 2 Humble、MoveIt Task Constructor、colcon
（`--symlink-install`）。

**Spec:** `src/Piper_ros/src/piper_elevator_manipulation/docs/press_button_demo_split.md`
—— 本计划从规格展开，执行者**两份都要读**。

---

## Global Constraints

逐字遵守，每个任务都隐含包含本节。

1. **零行为变化。** 不改任何默认值、范围、日志文本、执行顺序、异常类型。
   搬家时**逐字复制**，不"顺手优化"。
2. **本次不提 stage 名常量。** 4 个 stage 名保持裸字符串
   （规格 §6 已决议）。它们会分散在 3 个模块里，是本设计已知的遗留脆弱点。
3. **不修 bug。** 规格 §7 列的 4 个缺陷本次一律不碰。
4. **不动的文件：** `free_motion_route.cpp`、`cartesian_line_planner.cpp`、
   `five_dof_ik.cpp`、`target_source.cpp`、`trajectory_executor.cpp`、
   `contact_monitor.cpp`、`press_travel_monitor.cpp`。
5. **不改对外接口。** 其他包、`launch/*.py`、`tools/*.sh` 一行都不改。
6. **不建库。** CMake 里仍只有 `add_executable(press_button_demo ...)`，
   新 `.cpp` 加进它的源列表。
7. **每个任务必须先删掉 `.cpp` 里的旧副本。** 两个副本并存时编译器**不会报错**，
   链接器会静默选中一个 —— 这是本计划最危险的陷阱。
8. **命名：** 头文件 `include/piper_elevator_manipulation/<name>.hpp`，
   实现在 `src/<name>.cpp`，`#pragma once`，包内既有风格。
9. **提交粒度：** 一个任务一次提交，提交信息写清"搬了什么、从哪搬到哪"。
10. **⚠️ 不许重排换行。** 搬家时**只允许改行首缩进**；每行的内容、顺序、换行位置
    原样保留。这是验证脚本 B（函数体逐字对比）能工作的前提。
11. **链接性要保住。** 文件作用域常量一律 `inline constexpr`；模块**内部**的辅助
    函数（如 `solveWaypoint`、`kRadiansToDegrees`）放进「具名 namespace 里的匿名
    namespace」，不让它们从 internal linkage 变成 external linkage。
12. **不动函数体。** 任何函数体**内部**需要改动，说明"纯搬家"的前提不成立 ——
    **停下报告**，不要为了编过而就地改。

**容器层命令**（用户坐在容器里，宿主机路径会 `No such file or directory`）：

```bash
cd /ws && colcon build --symlink-install --packages-select piper_elevator_manipulation
```

---

## Review Focus

纯机械搬家，没有单测可写。以下 5 种"编译器抓不到、只有跑起来才暴露"的失效模式，
按可能性排序 —— 每个任务完成时逐条自查：

1. **新旧副本并存。** 新模块建好了，`.cpp` 里的旧定义忘了删。编译通过、链接通过、
   行为可能正确（也可能调用了旧的），**没有任何报错**。→ 每任务必查 §"搬家后自查"。
2. **`constexpr` 变量在头文件里的链接性。** 不加 `inline` 则每个 TU 各有一份，
   在 C++17 下多数情况能编过，但会埋 ODR 隐患。→ 统一用 `inline constexpr`。
3. **模板函数被放进 `.cpp`。** `getOrDeclareParameter` 是模板，**必须留在头文件里**；
   放 `.cpp` 会在其他 TU 链接时炸。
4. **漏 `#include`，靠传递包含侥幸编译。** 单包编译能过，换个包含顺序或增量重建就炸。
   → 每个新 `.hpp` 自己 include 它需要的东西，不依赖调用者先 include。
5. **匿名 namespace 的可见性。** 原代码在匿名 namespace 里，搬到新文件后必须包进
   `namespace piper_elevator_manipulation`，否则同名符号在多 TU 之间冲突。
6. **链接性悄悄变了。** 编译链接都过、文本也逐字相同，但文件作用域的**可变**全局/
   静态变量会从「每个 TU 一份」变成「全程序一份」—— 那是真的行为变化，且**没有任何
   报错**。→ 任务 1 里确认 `press_button_demo.cpp` 没有这类变量
   （通读时没看到，但必须验）。

---

## File Structure

| 文件 | 职责 | 从 `.cpp` 搬来的内容（原行号） |
|---|---|---|
| `include/.../press_constants.hpp` | 跨模块共享的字面量 | 47–51 |
| `include|src/.../press_solution.{hpp,cpp}` | MTC 解树 → 可执行/可打印的东西 | 86–89, 91–122, 124–140, 185–190 |
| `include|src/.../joint_state_cache.{hpp,cpp}` | 读实测关节角并判断到位/跟随 | 217–292, 294, 305–384 |
| `include|src/.../press_parameters.{hpp,cpp}` | 按压请求的类型、获取、校验 | 53–77, 142–215, 810–861, 872–912 |
| `include|src/.../press_task_builder.{hpp,cpp}` | 请求 → 未规划的 MTC 任务树 | 79–84, 386–400, 402–612 |
| `include|src/.../press_executor.{hpp,cpp}` | 按相序执行已规划的解 | 614–786 |
| `include|src/.../press_runner.{hpp,cpp}` | 一次完整流程的编排 | 863–870, 914–1076 |
| `src/press_button_demo.cpp` | 只剩 `main` | 1080–1288 |

**依赖方向（无环）：**

```
press_constants.hpp
   ↑            ↑              ↑
press_solution  press_task_builder   press_executor ← joint_state_cache
                    ↑                     ↑
              press_parameters ───────────┘
                    ↑
              press_runner  ←── 所有人
                    ↑
              main
```

---

## 验证方法

**A + B 必做，C 可选。** 没有机械臂也能完整验证 A 和 B。

| | 手段 | 证明什么 | 何时跑 |
|---|---|---|---|
| **A** | `colcon build` | 声明、引用、链接都对 | 每个任务 |
| **B** | `verify_pure_move.py` 函数体逐字对比 | **搬过去的东西一个字都没改** | 每个任务 |
| C | 仿真干跑 diff | 端到端行为一致 | 收尾，Gazebo 起得来就跑 |

### A — 编译

```bash
# 容器内
cd /ws && colcon build --symlink-install --packages-select piper_elevator_manipulation
```

### B — 主判据

**"纯搬家"的定义就是每个函数体逐字不变。B 直接检验这个定义。**

`scripts/verify_pure_move.py` 的做法：

1. 从 `git show <ref>:src/press_button_demo.cpp` 按**花括号配对**抽出 27 个待搬符号
   （21 个函数 + 6 个类型）的完整定义
2. 从当前 `src/` 下所有 `.cpp` / `include/**/*.hpp` 里抽同名定义
3. **规范化行首缩进后逐行对比**

```
$ python3 scripts/verify_pure_move.py --before HEAD
collectMotions              IDENTICAL   (press_solution.cpp)
createTask                  IDENTICAL   (press_task_builder.cpp)
solveWaypoint               IDENTICAL   (press_task_builder.cpp)
PressTaskParameters         IDENTICAL   (press_parameters.hpp)
main                        IDENTICAL   (press_button_demo.cpp)
...
27 identical, 0 different, 0 missing
```

三种结论：`IDENTICAL` = 通过；`DIFFERENT` = **失败，停下定位**；
`MISSING` = 找不到 —— 也失败。

**为什么 B 比干跑强**：干跑只走一条执行路径。`createRecoveryTask`、
`readPressParamsFile` 的出错分支、`waitForSettledJoints` 的超时分支、
`executePressSequence` 的每一个 `return 1`、`five_dof_ik` 的每个失败路径 ——
**一次都不会被执行，也就一次都不会被验证**。B 覆盖每一个函数体，
包括永远跑不到的分支。

**B 的前提**：不许重排换行（Global Constraint 10）。

### C — 可选，收尾时跑

没有机械臂，**不要**走 `press_button_hardware.launch.py`（需要 `can0`）。
走仿真：

```bash
# 容器内
ros2 launch piper_elevator_manipulation press_button_demo.launch.py \
  execute:=false rviz:=false 2>&1 | tee /tmp/sim_dryrun.log
```

⚠️ 必须在**改动前**用同一条命令取一份基线，改动后 diff。

---

## Task 1：验证工具 + 改动前快照

**Files:**
- Create: `scripts/verify_pure_move.py`
- Create: `docs/press_button_demo_split_symbols.txt`（27 个符号名，脚本的输入）

**Interfaces:**
- Produces: `python3 scripts/verify_pure_move.py --before <git-ref> [--symbols <file>]`，
  退出码 0 = 全部 IDENTICAL，1 = 有 DIFFERENT/MISSING
- Consumes: 无

- [ ] **Step 1.1：写符号清单**

`docs/press_button_demo_split_symbols.txt`，每行一个名字：

```
# 21 个函数
collectMotions
motionTimeout
solutionUsesStage
getOrDeclareParameter
speedScaling
freeMotionScaling
plannedSeconds
vector3Parameter
pressTimeout
lastCommandedPositions
logJointTracking
waitForSettledJoints
solveWaypoint
createTask
createRecoveryTask
executePressSequence
readPressParamsFile
validatePressParameters
waitForTarget
runSinglePress
main
# 6 个类型
PressTaskParameters
PlanningTask
PlannedMotion
JointStateCache
EditablePressParams
RunSettings
```

- [ ] **Step 1.2：写 `scripts/verify_pure_move.py`**

核心逻辑（约 130 行，只用标准库）：

1. `git show <ref>:src/piper_elevator_manipulation/src/press_button_demo.cpp`
   取"改动前"全文
2. 从文本里找每个符号的定义起点（正则：行首、可选的模板/返回类型、符号名、`(` 或
   `{`），再用**花括号配对**（跳过字符串与注释）找到定义终点
3. 当前文件：扫 `src/*.cpp` + `include/**/*.hpp`
4. 规范化：每行 `strip()`，丢弃空行
5. 逐行对比，打印结论；退出码汇总

⚠️ 脚本自身要能处理：类内联成员、模板函数、多行签名。

- [ ] **Step 1.3：自检 —— 对"改动前"的副本跑，应全部 IDENTICAL**

```bash
cd /ws/src/Piper_ros/src/piper_elevator_manipulation
# 把 HEAD 版本落到临时目录，当作"当前文件"来验脚本本身
git show HEAD:src/piper_elevator_manipulation/src/press_button_demo.cpp > /tmp/before.cpp
python3 scripts/verify_pure_move.py --before HEAD --symbols docs/press_button_demo_split_symbols.txt \
  --only /tmp/before.cpp
```
预期：`27 identical, 0 different, 0 missing`，退出码 0。

**若这里就不对，脚本有 bug —— 先修脚本，别往下走。**

- [ ] **Step 1.4：确认没有文件作用域的可变全局变量**

```bash
grep -nE '^[A-Za-z_].*=' src/press_button_demo.cpp | grep -vE 'constexpr|using |^.*\(' | head
```
预期：**无匹配**（Review Focus 第 6 条）。
若发现可变全局/静态变量，**停下报告** —— 它会在切分后悄悄改行为。

- [ ] **Step 1.5：对当前代码跑一次基线检查**

```bash
python3 scripts/verify_pure_move.py --before HEAD --symbols docs/press_button_demo_split_symbols.txt
```
预期：`27 identical, 0 different, 0 missing`。

- [ ] **Step 1.6：提交**

```bash
cd /ws/src/Piper_ros && git add src/piper_elevator_manipulation/scripts/verify_pure_move.py \
  src/piper_elevator_manipulation/docs/press_button_demo_split_symbols.txt \
  src/piper_elevator_manipulation/docs/press_button_demo_split.md \
  src/piper_elevator_manipulation/docs/press_button_demo_split_plan.md
git commit -m "press_button_demo 切分 1/4：验证工具 + 符号清单"
```

---

## Task 2：叶子层（三个互不依赖的模块）

**Files:**
- Create: `include/piper_elevator_manipulation/press_constants.hpp`
- Create: `include|src/piper_elevator_manipulation/press_solution.{hpp,cpp}`
- Create: `include|src/piper_elevator_manipulation/joint_state_cache.{hpp,cpp}`
- Modify: `src/press_button_demo.cpp`（删 47–51、86–140、185–190、217–292、294、305–384）

**Interfaces:**
- Produces：
  - `press_constants.hpp` → `kArmGroup`、`kEndEffectorLink`、`kPlanningFrame`、`kArmJoints`
    （全部 `inline constexpr`）
  - `press_solution.hpp` → `PlannedMotion`、`collectMotions`、`solutionUsesStage`、
    `plannedSeconds`、`motionTimeout`
  - `joint_state_cache.hpp` → `JointStateCache`、`lastCommandedPositions`、
    `logJointTracking`、`waitForSettledJoints`
- Consumes: 无（这三个都是叶子模块）

- [ ] **Step 2.1：`press_constants.hpp`**

```cpp
#pragma once

#include <array>

namespace piper_elevator_manipulation {

// Names shared by every module that talks to MoveIt or MTC about this arm.
// They were file-local constants in press_button_demo.cpp before the split.
inline constexpr char kArmGroup[] = "arm";
inline constexpr char kEndEffectorLink[] = "press_tcp";
inline constexpr char kPlanningFrame[] = "world";
inline constexpr std::array<const char*, 5> kArmJoints{
    "joint1", "joint2", "joint3", "joint4", "joint5"};

}  // namespace piper_elevator_manipulation
```

⚠️ 必须 `inline`（Global Constraint 11）。

- [ ] **Step 2.2：删掉 `.cpp` 里的 4 个常量定义，加 include + 临时 `using`**

```cpp
using piper_elevator_manipulation::kArmGroup;
using piper_elevator_manipulation::kArmJoints;
using piper_elevator_manipulation::kEndEffectorLink;
using piper_elevator_manipulation::kPlanningFrame;
```

⚠️ 这四行是**过渡态**，Task 4 结束时必须全部消失。

- [ ] **Step 2.3：`press_solution.{hpp,cpp}`**

逐字搬 `PlannedMotion`(86–89)、`collectMotions`(91–114)、`motionTimeout`(116–122)、
`solutionUsesStage`(124–140)、`plannedSeconds`(185–190)。

⚠️ `collectMotions` 里 `getWayPointCount() < 2` 那句**原样保留** ——
它正是"按压段消失"的现场，不要动（规格 §7）。

- [ ] **Step 2.4：删掉 `.cpp` 里的 `press_solution` 旧副本**

- [ ] **Step 2.5：`joint_state_cache.{hpp,cpp}`**

逐字搬 `JointStateCache`(217–292)、`lastCommandedPositions`(305–318)、
`logJointTracking`(320–350)、`waitForSettledJoints`(352–384)。

⚠️ `kRadiansToDegrees`(294) 只在模块内部用 → 放**「具名 namespace 里的匿名 namespace」**，
保住 internal linkage（Global Constraint 11）。

- [ ] **Step 2.6：删掉 `.cpp` 里的 `joint_state_cache` 旧副本**

- [ ] **Step 2.7：编译（A）**

```bash
cd /ws && colcon build --symlink-install --packages-select piper_elevator_manipulation
```

- [ ] **Step 2.8：函数体逐字对比（B）**

```bash
cd /ws/src/Piper_ros/src/piper_elevator_manipulation
python3 scripts/verify_pure_move.py --before HEAD --symbols docs/press_button_demo_split_symbols.txt
```
预期：27 个符号**全部 `IDENTICAL`**（搬走的在别处找到，没搬的还在 `.cpp` 里）。
**任何 `DIFFERENT` / `MISSING` → 停下定位，不要提交。**

- [ ] **Step 2.9：搬家后自查（Review Focus 第 1 条 —— 新旧副本并存）**

```bash
cd /ws/src/Piper_ros/src/piper_elevator_manipulation/src
grep -nE '^(void collectMotions|std::chrono::milliseconds motionTimeout|bool solutionUsesStage|double plannedSeconds|class JointStateCache|auto lastCommandedPositions|void logJointTracking|void waitForSettledJoints)|constexpr char kArmGroup' press_button_demo.cpp
```
**预期：无输出。**

- [ ] **Step 2.10：提交**

```bash
cd /ws/src/Piper_ros && git commit -am "press_button_demo 切分 2/4：叶子层（常量 / 解树 / 关节角）"
```

---

## Task 3：中间层（参数 → 构建 → 执行）

**Files:**
- Create: `include|src/piper_elevator_manipulation/press_parameters.{hpp,cpp}`
- Create: `include|src/piper_elevator_manipulation/press_task_builder.{hpp,cpp}`
- Create: `include|src/piper_elevator_manipulation/press_executor.{hpp,cpp}`
- Modify: `src/press_button_demo.cpp`
  （删 53–77、79–84、142–215、386–400、402–612、614–786、810–861、872–912）

**Interfaces:**
- Consumes: `press_constants`、`press_solution`、`joint_state_cache`（Task 2）
- Produces：
  - `press_parameters.hpp` → `PressTaskParameters`、`EditablePressParams`、
    `readPressParamsFile`、`validatePressParameters`、`getOrDeclareParameter`（**模板**）、
    `speedScaling`、`freeMotionScaling`、`vector3Parameter`、`pressTimeout`
  - `press_task_builder.hpp` → `PlanningTask`、`createTask`、`createRecoveryTask`
  - `press_executor.hpp` → `executePressSequence`

⚠️ **这是四块里最大的一块**（~610 行）。三个模块有内部依赖
（executor → builder → parameters），一次搬完，最后统一编译 + 验证。

- [ ] **Step 3.1：`press_parameters.{hpp,cpp}`**

逐字搬 `PressTaskParameters`(53–77)、`getOrDeclareParameter`(142–156)、
`speedScaling`(158–170)、`freeMotionScaling`(172–183)、`vector3Parameter`(192–202)、
`pressTimeout`(204–215)、`EditablePressParams`(810–818)、`readPressParamsFile`(820–861)、
`validatePressParameters`(872–912)。

⚠️ **`getOrDeclareParameter` 是模板 —— 整个函数体必须留在 `.hpp` 里**
（Review Focus 第 3 条）。

规格 §7 列的 4 个缺陷都在这段里，**本次一律不改**。

- [ ] **Step 3.2：`press_task_builder.{hpp,cpp}`**

逐字搬 `PlanningTask`(79–84)、`solveWaypoint`(386–400)、`createTask`(402–563)、
`createRecoveryTask`(565–612)。

⚠️ 模块**内部**的辅助函数不进头文件 —— 放「具名 namespace 里的匿名 namespace」
（Global Constraint 11）。

- [ ] **Step 3.3：`press_executor.{hpp,cpp}`**

逐字搬 `executePressSequence`(614–786)。

⚠️ 它依赖前两个模块 —— 三个一起编，不要单独提前编。

- [ ] **Step 3.4：删掉 `.cpp` 里这三个模块的全部旧副本**

- [ ] **Step 3.5：`.cpp` 加 include + 过渡 `using`**

- [ ] **Step 3.6：编译（A）**

```bash
cd /ws && colcon build --symlink-install --packages-select piper_elevator_manipulation
```

- [ ] **Step 3.7：函数体逐字对比（B）**

```bash
cd /ws/src/Piper_ros/src/piper_elevator_manipulation
python3 scripts/verify_pure_move.py --before HEAD --symbols docs/press_button_demo_split_symbols.txt
```
预期：**27 个符号全部 `IDENTICAL`**，其中约 15 个现在应报告位置在新模块里。

- [ ] **Step 3.8：搬家后自查（Review Focus 第 1 条）**

```bash
cd /ws/src/Piper_ros/src/piper_elevator_manipulation/src
for s in createTask createRecoveryTask solveWaypoint executePressSequence \
         readPressParamsFile validatePressParameters; do
  grep -qn "^[A-Za-z_].*\b$s(" press_button_demo.cpp && echo "⚠️ 残留: $s"
done
grep -nE '^(struct PressTaskParameters|struct PlanningTask|struct EditablePressParams|template <typename T>|double speedScaling|double freeMotionScaling|Eigen::Vector3d vector3Parameter|std::chrono::milliseconds pressTimeout)' press_button_demo.cpp
echo "自查完毕"
```
**预期：只有 `自查完毕`，没有任何 `⚠️ 残留`。**

- [ ] **Step 3.9：提交**

```bash
cd /ws/src/Piper_ros && git commit -am "press_button_demo 切分 3/4：中间层（参数 / 任务构建 / 执行）"
```

---

## Task 4：编排层 + 收尾 + 全量验证

**Files:**
- Create: `include|src/piper_elevator_manipulation/press_runner.{hpp,cpp}`
- Modify: `src/press_button_demo.cpp`（删 863–870、914–1076；清掉残留空壳和过渡 `using`）
- Modify: `CMakeLists.txt`（`add_executable` 源列表补齐）

**Interfaces:**
- Consumes: 前面所有模块 + `piper_elevator_manipulation::TargetSource`
- Produces：
  - `struct RunSettings { bool execute; std::string execution_mode; bool hardware_enable;
    bool simulated_contact; bool use_sim_time; double travel_margin; };`
  - `std::optional<piper_elevator_manipulation::PressTarget> waitForTarget(
    const rclcpp::Node::SharedPtr&, piper_elevator_manipulation::TargetSource&, double timeout_s);`
  - `int runSinglePress(const rclcpp::Node::SharedPtr&, const RunSettings&,
    const PressTaskParameters&, JointStateCache&)`

- [ ] **Step 4.1：`press_runner.{hpp,cpp}`**

逐字搬 `RunSettings`(863–870)、`waitForTarget`(914–955)、`runSinglePress`(957–1076)。

- [ ] **Step 4.2：删掉 `.cpp` 里的旧副本**（863–870、914–1076）

- [ ] **Step 4.3：清理 `press_button_demo.cpp`**

此时它应该只剩：include 区、`namespace mtc/stages/solvers` 别名、`using` 声明、`main`。
删掉所有残留的 `namespace { ... }` 空壳和**全部过渡 `using`**。

- [ ] **Step 4.4：核对文件规模**

```bash
wc -l /ws/src/Piper_ros/src/piper_elevator_manipulation/src/press_button_demo.cpp
```
预期：**~210 行**。若 > 300 行，说明有东西没搬干净。

- [ ] **Step 4.5：CMake 源列表补齐**

`CMakeLists.txt` 的 `add_executable(press_button_demo ...)` 加上 7 个新 `.cpp`。
⚠️ 确认没有重复项、没有漏项。

- [ ] **Step 4.6：全量编译（A，干掉增量缓存）**

```bash
cd /ws && rm -rf build/piper_elevator_manipulation install/piper_elevator_manipulation \
  && colcon build --symlink-install --packages-select piper_elevator_manipulation
```
（`rm -rf` 排除增量构建掩盖问题，也顺手规避 memory 里记的
"colcon 只复制、从不删除"的老坑。）

- [ ] **Step 4.7：🚨 全量逐字对比（B，核心判据）**

```bash
cd /ws/src/Piper_ros/src/piper_elevator_manipulation
python3 scripts/verify_pure_move.py --before HEAD --symbols docs/press_button_demo_split_symbols.txt
```
预期：**`27 identical, 0 different, 0 missing`**，退出码 0。

- [ ] **Step 4.8：残留定义总清查**

```bash
cd /ws/src/Piper_ros/src/piper_elevator_manipulation/src
for s in collectMotions motionTimeout solutionUsesStage plannedSeconds \
         getOrDeclareParameter speedScaling freeMotionScaling vector3Parameter pressTimeout \
         JointStateCache lastCommandedPositions logJointTracking waitForSettledJoints \
         solveWaypoint createTask createRecoveryTask executePressSequence \
         readPressParamsFile validatePressParameters waitForTarget runSinglePress; do
  grep -q "^[A-Za-z_].*\b$s(" press_button_demo.cpp && echo "⚠️ 残留: $s"
done; echo "自查完毕"
```
**预期：只有 `自查完毕`，没有任何 `⚠️ 残留`。**

- [ ] **Step 4.9：确认产物没变多**

```bash
ls /ws/install/piper_elevator_manipulation/lib/piper_elevator_manipulation/
```
预期：只有 `press_button_demo` 和 `virtual_contact_sim`。

- [ ] **Step 4.10：确认对外零影响**

```bash
cd /ws/src/Piper_ros && git status --short
```
预期：改动只在 `piper_elevator_manipulation/` 包内。
**若 `tools/`、`launch/` 或其他包出现改动 → 停下来报告。**

- [ ] **Step 4.11：C（可选）—— 仿真干跑 diff**

⚠️ 没有机械臂，**不要**走 `press_button_hardware.launch.py`（需要 `can0`）。走仿真：

```bash
ros2 launch piper_elevator_manipulation press_button_demo.launch.py \
  execute:=false rviz:=false 2>&1 | tee /tmp/sim_after.log
```
与改动前用**同一条命令**取的基线 diff。起不来就跳过 —— **A + B 已经是充分证据。**

- [ ] **Step 4.12：提交**

```bash
cd /ws/src/Piper_ros
git add src/piper_elevator_manipulation && git commit -m "press_button_demo 切分 4/4：编排层 + main 瘦身 + CMake"
```

- [ ] **Step 4.13：收尾 —— 先推内层，再跟外层指针**

```bash
# 容器内 → 先推内层
cd /ws/src/Piper_ros && git push
```
```bash
# 宿主机 → 再跟外层指针（顺序不能反，见 memory）
cd ~/piper_ws && git add src/Piper_ros \
  && git commit -m "子模块：press_button_demo 切分完成" && git push
```

---

## 失败处理

| 症状 | 处置 |
|---|---|
| `verify_pure_move.py` 报 `DIFFERENT` | **不要提交。** 因为不许重排换行，差异应精确落在改动点上 —— `git diff` 一看就知道。绝大多数是搬家时手滑改了内容 |
| 报 `MISSING` | 符号既不在 `.cpp` 也不在新模块里 —— 搬丢了，或者名字打错 |
| 编译失败报"找不到符号" | 查 Review Focus 第 1、4 条：旧定义删了没、新头文件自己 include 了没 |
| 编译链接都过、B 也全绿，但行为可疑 | 查 Global Constraint 11 的链接性问题；并跑 C（仿真干跑）|
| 中途发现某个符号跨模块共享、原计划没料到 | 停下来说明，**升级回规格**再继续（技能规定：路径只能升不能降）|
| 某任务做完发现设计不对想回退 | `git reset --hard` 到上一个任务 —— 一个任务一次提交就是为了这个 |
