# piper_marker_perception

识别 ArUco 标定板，解出板子位姿，再换算成规划层要的按压目标。

**当前状态：离线部分已完成并通过验证；ROS 节点还没写。**
见 `piper_elevator_manipulation/docs/marker_perception_plan.md`。

---

## 现在有什么

| 文件 | 作用 |
|---|---|
| `scripts/aruco_board.py` | 核心库：板子几何、检测、`solvePnP`、内参读取 |
| `scripts/press_targets.py` | 核心库：板子→按键的固定变换、四元数构造 |
| `scripts/make_board.py` | 生成**可打印**的板子图（含 100 mm 校准刻度线）|
| `scripts/detect_board.py` | 离线命令：吃一张图 → 出位姿 + 重投影误差 |
| **`scripts/press_target_bridge.py`** | ⭐ **发布 `/press/target_pose`** —— 发布板子位姿进来，按键目标出去 |
| **`scripts/teach_press_offset.py`** | 教示：用机械臂测出按键相对板子的位置和朝向 |
| `scripts/selftest_offline.py` | 合成图自测（板子识别），不需要硬件 |
| `scripts/selftest_bridge.py` | 离线自测（板子→按键几何），不需要硬件 |
| `config/board.yaml` | 板子定义 **+ 板子→按键的固定关系** |

**还没有**：`camera_publisher`、`marker_locator`（把相机图像变成
`/perception/board_pose` 的那个节点）。

```
现在能测：                                   还差：
                                           
  /perception/board_pose ──► 桥 ──► /press/target_pose
      ↑ 假的也行                             ↑ 验证过了
                                           
  相机 ──► marker_locator ──┘
           ↑ 这个还没写
```

---

## 快速开始（现在就能做，不需要相机/机械臂）

### 1. 生成板子并打印

```bash
cd /ws/src/Piper_ros/src/piper_marker_perception
python3 scripts/make_board.py --out ~/aruco_board.png
```

打印时设 **100% / 实际大小**，不要「适应页面」。

印完做两件事：

1. **量图下方的刻度线** —— 应该是 100.0 mm。不是就说明打印缩放不对。
2. **拿卡尺量一个 marker 的边长**，填回 `config/board.yaml` 的 `marker_length`；
   同样量一下 marker 之间的**空隙**填 `marker_separation`。

> ⚠️ 这两步不能省。家用打印机有 ±1% 缩放，110 mm 的板子可能差 1 mm，
> 而 1% 的尺度误差会原样变成 1% 的位置误差 —— 在 0.4 m 处就是 4 mm。

### 2. 先跑自测确认代码没问题

```bash
python3 scripts/selftest_offline.py
```

会在合成图上验证板系约定和位姿恢复精度（不需要相机）。

### 3. 拿手机/相机拍几张板子，跑离线检测

```bash
python3 scripts/detect_board.py ~/photos \
    --camera-info /path/to/camera_info.yaml \
    --out ~/photos_debug
```

输出每张图的 marker 数、重投影 RMS、距离、平移量；`--out` 里的调试图画了
角点、ID 和板系坐标轴（红 +X / 绿 +Y / 蓝 +Z，蓝轴应指向相机）。

**调检测参数就用这个**：拍十来张真实距离和角度的照片，一次跑完，
看什么条件下 marker 数和重投影误差开始变差 —— 比移动机械臂快得多也稳定得多。

---

## 板子 → 按键：那个固定的位姿关系

板子不是要按的东西，它是**能看到的东西**。整条链路靠一个物理前提：

> **板子和按键刚性固定在同一块面板上。**

在这个前提下，按键的位置和朝向由板子的位姿经过一个**常量变换**得到：

$$
T_{base \leftarrow 按键} = T_{base \leftarrow 板子} \cdot \underbrace{T_{板子 \leftarrow 按键}}_{\text{常量，装好后不变}}
$$

这个常量在 `board.yaml` 里分成**两块**存（因为实际就是这么量出来的）：

| 字段 | 含义 | 谁能量 |
|---|---|---|
| `tip_in_board` | 杆尖**应该落到的点**，板系，米 | 卷尺能估，教示更准 |
| `press_axis_in_board` | 按压方向（指向按键内部），板系单位向量 | **只有教示能量** |

默认按压轴是 `(0, 0, -1)` = 板系 −Z。板子和按键共面时这就是对的（两者法线相同）。
只有按键行程轴和面板法线不一致时才需要改 —— 那种按键**斜着按会卡死**，
所以方向是真的输入，不是形式。

### 教示（推荐，比卷尺准一个量级）

```bash
# 需要感知节点在跑且此刻能看到板子
ros2 run piper_marker_perception teach_press_offset.py --name red --poses 3
```

手动把杆尖**顶到按键中心**，回车。脚本记录那一刻：

```
T_base←板子      来自感知节点
T_base←press_tcp 来自 TF（杆尖真实位置）
```

于是

$$
tip_{板系} = R_{base \leftarrow 板子}^{\mathsf{T}} \cdot (p_{tcp} - t_{base \leftarrow 板子})
$$

而**按压轴是免费的**：杆尖顶在按键上时，工具的 +Z **就是**操作者用小臂瞄的方向，
取同一个变换就得到它在板系下的表示。

`--poses 3` 让它在 3 个不同臂位姿各测一次。那个偏移是个**常量**，三次结果必须一致 ——
不一致说明问题不在教示，而在**手眼标定**（那是三次之间唯一变化的东西）。
这把标定质量变成现场可读的数。

输出是一段 YAML，**手动贴回** `config/board.yaml`。脚本不直接改文件，因为那会
把解释这些数字含义的注释一起冲掉，而那些注释是文件价值的大部分。

### 桥节点

```bash
ros2 run piper_marker_perception press_target_bridge.py --ros-args -p target:=red
```

> ⚠️ ROS 参数**必须**跟在 `--ros-args` 后面。否则 rclpy 会把 `target:=red`
> 当成 remap 规则，参数根本没传进去，然后报"缺少 target 参数"，非常费解。

会用到的参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `target` | — | **必填**，按键名 |
| `config` | 包内 `board.yaml` | 配置路径 |
| `board_topic` | `/perception/board_pose` | 输入 |
| `topic` | `/press/target_pose` | 输出 |
| `publish_rate_hz` | 5.0 | 持续发布（契约允许持续发）|
| `max_age_s` | 1.0 | 板子位姿超过这个年龄就停发 |
| `require_taught` | true | `taught: false` 时拒绝发布 |
| `direction_sign` | 1.0 | 和规划层同名参数，给存了外法线的人 |

它会拒掉三类输入，**每类只在状态变化时报一次**（不然 5 Hz 会刷屏）：

| 拒绝什么 | 为什么 |
|---|---|
| `taught: false` | 配置里是占位值，拿它去按压 = 按到不知道哪里 |
| 板子位姿在相机系 | 规划层只认 `world` / `base_link`，相机系会被它静默拒收 |
| 目标在工作空间盒外 | 提前用和规划层**同一组边界**拦下，报错清楚得多 |
| 板子位姿超过 `max_age_s` | 停止发布，避免拿旧位姿指挥 |

### 不接相机怎么测桥

```bash
python3 scripts/selftest_bridge.py        # 只要 numpy，连 ROS 都不要
```

它验证四元数和 `send_press_target.py` **逐位一致**（那是已经跑通过的参考实现），
以及一组手算过的例子。桥的 ROS 层另外用假板子位姿做过集成验证。

---

## 板系约定（钉死在代码里）

```
原点 = 板子几何中心
+X  向右（面向板子看）
+Y  向上
+Z  垂直板面向外，指向观察者
```

⇒ **按压方向 = 板系 −Z**。

`solvePnP` 给出的位姿是 `T_camera_optical <- board`，用的就是这个坐标系。
打印出来的板子贴在面板上、正面朝机械臂时，蓝轴（板系 +Z）应指向相机。

---

## ⚠️ 三条必须遵守的事

### 1. 180° 旋转：三处必须一致

相机传感器倒装，**原始图是上下颠倒的**。这里**故意不纠正**它 —— 那 180° 被
**吸收进手眼标定结果**里了。所以三处必须用同一套：

```
capture_session.py     存原始帧
标定                    在原始帧上求内参和手眼
aruco_board.py         在原始帧上检测
```

**任何一处单独旋转，结果就错一个固定角度，而且看起来"差不多对"。**

### 2. 板子贴反了不会被发现 —— 只有 marker 数量能抓

这是实测出来的，不是推测：

| 情况 | marker 数 | 重投影 RMS | 解出的位姿 |
|---|---|---|---|
| 正常正对 | 9/9 | 0.21 px | 正确 |
| **贴反** | **1/9** | **0.14 px** | **看起来完全正常，但错** |

原因：从背面看，每个 marker 都是镜像的，而镜像的 ArUco 图案不是合法编码，
所以 8 个直接解不出来。但**剩下的 1 个足够 `solvePnP` 拟合出一个朝前、
指向正确、重投影误差很低的位姿** —— 而且是错的。

**⇒ `min_markers` 必须设成接近板子上 marker 的总数**（本板子 9 个，建议 ≥ 4，
严格一点用 ≥ 6）。这是唯一可靠的信号。

### 3. 板子尺寸填实测值

见上面「快速开始」第 1 步。

---

## 配置

### `config/board.yaml`

```yaml
board:
  dictionary: DICT_4X4_50     # 容器里 25 种字典都可用
  markers_x: 3
  markers_y: 3
  marker_length: 0.030        # ★ 实测值，米
  marker_separation: 0.010    # ★ 实测值（两个 marker 之间的空隙）
  first_marker_id: 0
  accepted_ids: [0,1,2,3,4,5,6,7,8]
```

板子总尺寸 = `3×30 + 2×10 = 110 × 110 mm`。

### 检测器参数

在 `aruco_board.py` 的 `DETECTOR_DEFAULTS` 里，可以按需覆盖：

```python
detector = spec.make_detector(cornerRefinementWinSize=7, adaptiveThreshConstant=5.0)
```

`cornerRefinementMethod = CORNER_REFINE_SUBPIX` **不要关** —— 它是精度影响
最大的一个参数，关掉之后位姿噪声大约差一个数量级。

---

## 环境事实（实测，2026-09-29）

| 项 | 结果 |
|---|---|
| OpenCV | **4.5.4**，`cv2.aruco` 用**旧版 API**（`DetectorParameters_create` / `GridBoard_create`）|
| `cv2.ArucoDetector` | ✗ 4.7+ 才有，用不了 |
| 字典 | 25 种预定义字典都可用；`DICT_4X4_50` 只有 **50 个 ID** |
| 可用函数 | `detectMarkers`、`estimatePoseSingleMarkers`、`estimatePoseBoard`、`estimatePoseCharucoBoard`、`solvePnP`、`solvePnPRansac`、`solvePnPRefineLM`、`calibrateHandEye` |
| 相机进容器 | ⚠️ 需要 `privileged: true`（已改 compose）+ `v4l-utils`（已加 Dockerfile）|

---

## 为什么不用 `cv2.aruco.GridBoard` 建 objPoints

`GridBoard` 对「板系原点在哪、轴朝哪」有自己的定义，而且**在 OpenCV 版本之间
变过**。`solvePnP` 只能对得起喂给它的点，所以 `aruco_board.py` 里**自己算**
object points。代价是对 marker 间距做点算术，收益是同一个位姿在任何一个
OpenCV 上含义都一样。
