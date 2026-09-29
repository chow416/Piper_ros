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
| `scripts/check_intrinsics.py` | **内参体检**：用之前先查一遍，抓重投影误差抓不到的错误 |
| **`scripts/marker_locator.py`** | ⭐ **实时识别**：开相机 → 检测 → 发布板子位姿 + 调试图 |
| `scripts/press_target_bridge.py` | 发布 `/press/target_pose` —— 板子位姿进来，按键目标出去 |
| `scripts/teach_press_offset.py` | 教示：用机械臂测出按键相对板子的位置和朝向 |
| `scripts/selftest_offline.py` | 合成图自测（板子识别），不需要硬件 |
| `scripts/selftest_bridge.py` | 离线自测（板子→按键几何），不需要硬件 |
| `config/board.yaml` | 板子定义 **+ 板子→按键的固定关系** |
| `config/marker_locator_*.yaml` | **每个相机一份**参数（设备、分辨率、内参）|

```
相机 ✅ ──► marker_locator ✅ ──► /perception/board_pose
                                      │
                                      ├─► press_target_bridge ✅ ──► /press/target_pose
                                      └─► 或只用来验证识别
```

### 快速跑起来

```bash
# 相机是独占的，先把 cam_view.py 关掉
python3 scripts/marker_locator.py \
    --ros-args --params-file config/marker_locator_orbbec.yaml
```

它会打印板子位姿，并把**画了检测框的实时画面**发到
`/perception/debug_image/compressed`：

```bash
ros2 run rqt_image_view rqt_image_view /perception/debug_image/compressed
```

默认发在**相机光学系**（`wrist_depth_camera_optical_frame`）—— 不需要机械臂、
不需要 TF、不需要手眼标定，插上相机就能跑。

要用机械臂系下的位姿，加上 `-p target_frame:=base_link`，那就需要
`robot_state_publisher` + `/joint_states` + 写进 URDF 的手眼标定，
**而且只有在机械臂停稳时才发**（相机在腕部，运动中时间误差会直接变成位置误差）。

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

## 换相机：只改配置，不改代码

相机的**全部差异**都在参数文件里，两个现成的：

| 文件 | 相机 |
|---|---|
| `config/marker_locator_orbbec.yaml` | Orbbec DaBai DCW2（腕部正式相机）|
| `config/marker_locator_c270.yaml` | Logitech C270（桌面测试用）|

```bash
ros2 run piper_marker_perception marker_locator.py \
    --ros-args --params-file <包>/config/marker_locator_c270.yaml
```

两者只差三处：

| 参数 | Orbbec | C270 |
|---|---|---|
| `device_hint` | `orbbec,dabai,sonix` | `logitech,046d:0825` |
| `width` / `height` | 1920×1080 | 640×480（跟那份标定一致）|
| `camera_info` | `camera_info_nominal.yaml` | `camera_info_c270_calibrated.yaml` ⚠️ |

⚠️ **`device_hint` 是必须的**。早期版本把 `"orbbec"` 写死在代码里，结果一个
完全正常的罗技摄像头永远找不到 —— 它的名字是 `UVC Camera (046d:0825)`。

### 分辨率与内参是一对，但不再靠人记住

请求的分辨率不一定拿得到：C270 最高只有 1280×960，填 `1920x1080` 会被
OpenCV **静默协商降级**，不报错。而内参只在测它的那个分辨率上成立 ——
640×480 的内参配 1280×960 的帧，所有焦距差 2 倍，**所有距离就差 2 倍**，
而且解出来的位姿看上去完全自洽。

所以 `marker_locator` 现在会**读回相机实际给的分辨率**：

- 和请求的不一致 → 告警
- 和内参记录的不一致 → 按比例换算 `fx`/`fy`/主点，并告警
  （畸变系数不变，它是归一化半径的函数；前提是两种分辨率覆盖同一视场）
- 内参没记录分辨率 → 告警说无法核对

启动时还会打印**视场角**：

```
[INFO] lens 15.8 x 10.3 deg, fx=2299.2 fy=2664.5
```

视场角是唯一能和物理常识对账的数 —— 网路摄像头上挂个 16° 的镜头一看就不对，
而 `fx=2299` 不查规格书看不出来。

### 配置路径

参数文件里写 `config/camera_info_c270_calibrated.yaml` 就够了，不用绝对路径。
同一个脚本在源码树里和 `ros2 run` 安装后所处的层级不同，`resolve_config_path`
会依次试：当前目录 → 脚本旁边 → 安装后的 share 目录。

（这个曾经是个真 bug：`DEFAULT_CONFIG` 按 `dirname(dirname(__file__))` 拼，
从源码跑没问题，一 `ros2 run` 就变成 `lib/config/board.yaml`。）

### ⚠️ C270 不是倒装的，Orbbec 是

之前反复确认过 **Orbbec 的传感器 180° 倒装**，而那 180° 被**吸收进手眼标定
结果**里 —— 整套约定（`capture_session.py` 存原始帧、标定用原始帧、识别用原始帧）
都是围绕它建立的，连标称内参也是按原始帧给的。

**C270 是正装的。** 所以：

- ✅ 拿 C270 跑识别、验证代码 —— 没问题，板子识别和 PnP 与安装方向无关
- ❌ **不能把两者的标定数据混用** —— 差一个 180°，也就是绕光轴转半圈

### C270 的已知噪音（无害）

```
Corrupt JPEG data: 3 extraneous bytes before marker 0xd3
```

罗技在 MJPG 帧尾部多塞了几个字节，libjpeg 会抱怨但**照常解码**。实测
1280×960 下 9/9 marker 全部检出。试过 `cv2.utils.logging.setLogLevel`
（OpenCV 4.5.4 没这个属性）和 `OPENCV_LOG_LEVEL` 环境变量（无效）——
这些消息是 libjpeg 直接写 stderr 的，没接进 OpenCV 的日志系统。压不掉，忍一下。

---

## 收到一份内参，先体检，别直接用

```bash
ros2 run piper_marker_perception check_intrinsics.py <内参文件> --lens c270
```

只做算术，不碰相机。查的是**重投影误差查不出来的那类错误**：五个畸变系数
加上焦距和主点，自由度比拟合所需的多，所以一个跑偏的求解器能把残差压到
0.1 px，同时让模型在拟合用的那几千个像素之外完全不可用。

`config/camera_info_c270_calibrated.yaml` 就是这样一个真实例子：

| 检查项 | 结果 |
|---|---|
| fx 与 fy | 相差 13.7%（真实传感器是方形像素，正常 <1%）|
| 视场 | 15.8° × 10.3°（C270 规格书 60° 对角 = 水平 49.6°）|
| 径向项抵消 | 画面四角处量级之和 0.1258，合起来只有 −0.0183，抵消掉 6.9 倍 |
| 径向因子 | 出画面 1.72 倍就翻号 —— 会折过光轴，真实镜头不会 |
| **重投影误差** | **0.128 px —— 好看，但它不能说明内参是对的** |

结论：焦距是真实值的 **3.32 倍**，所以**所有距离被放大 3.32 倍**。
实机验证过：板子放在约 0.49 m 处，节点读出 **1.63 m**。乘以 3.32 正好。

这类结果几乎总是一个原因：标定时棋盘姿态不够多样，径向畸变没有被真正观测到，
求解器就把焦距和畸变互相补偿掉了。重标定时图像 ≥15 张、棋盘要倾斜、
要出现在画面四角、要占满画面。

⚠️ 这不是「PnP 坏了」。PnP 会忠实地按你给的内参算 —— 内参错了，它给出的
也是一个自洽的、整体错尺度的位姿。所以**唯一能在现场抓出它的办法是量距离**。

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
