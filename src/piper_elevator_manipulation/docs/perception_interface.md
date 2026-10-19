# 按压目标接口说明（感知层 → 规划控制层）

> 本文用中文写，是给做感知/上位机的同事直接看的工作约定。
> 代码和字段名保持英文，方便对照。
> 对应实现：`src/target_source.cpp`、`scripts/send_press_target.py`。

## 语义检测接入（当前推荐）

`external/robot-arm-perception` 的最终输出不是直接写入本接口，而是先发布：

```text
/perception/button_targets_base     vision_msgs/msg/Detection3DArray
```

临时接入只信任其中已经处于 `base_link` 的位置
`results[0].pose.pose.position`，并保留稳定 `id`、语义标签（例如 `floor_4`）和
观测时间。相机内外参、手眼和深度面法向尚未确认，因此输入四元数当前**不参与**
规划；不要在这里再做一次手眼变换或坐标补偿。

`button_target_selector.py` 用固定的 `target_semantic` 选择一个目标，并用启动参数
`fixed_press_direction_x/y/z` 构造姿态：输出 pose 的 `+Z` 恒等于该固定、向内的面板
法向（默认 `+X`）。它把位置加上这个姿态转发为一次 `/press/target_pose` 粗目标，
以及后续同 ID 的 `/press/refine_pose` 精修正目标。首次同语义多目标、ID 丢失后多候选、
坐标系不对、时间戳无效/过期/过早，都会拒绝而不是猜测。规划节点继续拥有最终
可达性、碰撞、停稳时间戳和精修正融合的决定权。

## 一句话

往 **`/press/target_pose`** 发一条 `geometry_msgs/msg/PoseStamped`，
机械臂就去按你给的那个点，按完自己退回原位。

## 1. 话题约定

| 项目 | 值 |
| --- | --- |
| 话题名 | `/press/target_pose` |
| 消息类型 | `geometry_msgs/msg/PoseStamped` |
| QoS | `RELIABLE`，`KEEP_LAST` `depth=1`，`VOLATILE` |
| 发布频率 | 不限。持续发也行，事件触发发一条也行，任务只取最新一条 |
| `header.frame_id` | 必须是 `world` 或 `base_link`，其它一律拒收 |

### 关于坐标系

**`world` 和 `base_link` 是同一个坐标系**，填哪个都一样。
URDF 里 `world → base_link` 是一个 origin 全零的固定关节，变换是单位矩阵。

所以可以直接理解成「**相对机械臂底座**」：

| 轴 | 方向 |
| --- | --- |
| `+X` | 前方（远离底座，朝工作区） |
| `+Y` | 左侧 |
| `+Z` | 上方 |

单位一律是**米**。

其它坐标系（相机系、标定系、`camera_color_optical_frame` 等）目前**会被拒绝**，
因为还没接 TF 变换。这个要单独排期，不是改个参数就能用的。

## 2. 字段怎么填

```text
PoseStamped
├── header
│   ├── stamp    观测时刻，必须填真实时间（判过期用）
│   └── frame_id "world" 或 "base_link"
└── pose
    ├── position      目标点，单位米
    │   ├── x
    │   ├── y
    │   └── z
    └── orientation   单位四元数 (x, y, z, w)
                      ★ 它的 +Z 轴 = 按压方向 ★
```

### `position` 的含义

是**按压杆尖端的落点**，也就是你想让杆尖碰到的那一点。
不是按钮中心、不是面板平面、不是机械臂末端法兰。

### `orientation` 的用法

任务节点**只从四元数里取 +Z 轴的方向**，另外两个轴（绕杆自转的滚转角）**完全忽略**。

原因：这是五轴臂，`joint6` 不参与运动，绕杆自转本来就是自由的。
IK 会自己在多组解里挑一个。

所以你们**不需要精确构造姿态**，只需要让 +Z 指向你要压的方向。

## 3. 按压方向怎么定

方向 `n` = 你希望杆尖**前进**的方向，即**从外侧指向按钮内部**。

例 1：按钮在竖直面板上，杆从正前方水平顶过去

```text
目标点  (0.300, 0.000, 0.240)  m
方向    (1, 0, 0)              → 往 +X 压
```

例 2：按钮在桌面上，杆竖直往下压

```text
方向    (0, 0, -1)             → 往 -Z 压
```

方向向量**不需要归一化**，随便给长度都行（会内部归一化），但不能是零向量。

> **⚠️ 最容易搞错的一点**：如果你算出来的是面板**外法线**（指向机械臂那一侧），
> 那它和这里的约定是**反的**。两种解决办法，任选：
> 1. 在你们那边取反再塞进四元数（推荐）；
> 2. 启动任务时加参数 `press_direction_sign:=-1`，让它自己翻。
>
> 方向错了的后果：机械臂往**远离**按钮的方向走，然后在空中做一遍按压动作。

## 4. 任务节点拿到目标后会做什么

设 `n` 为单位按压方向，目标点为 `p`：

```text
对准点 = p − approach_distance × n     ← 先在表面外 30 mm 处对准
按压终点 = p + press_depth × n         ← 再往里压 3 mm
```

默认 `approach_distance = 0.030 m`，`press_depth = 0.003 m`（实机档）。
两个值都是任务侧参数，不用感知层管。

完整动作：**自由段接近 → 直线对准 → 按压 → 退回 → 回原位**。
任务结束后机械臂回到它开始的那一点，不会停在按钮上。

## 5. 哪些消息会被拒绝

被拒绝的消息**完全不会影响机械臂**，只会记数并打日志。

| 情况 | 日志里的原因 |
| --- | --- |
| `frame_id` 为空 | `header.frame_id is empty` |
| `frame_id` 不是 world/base_link | `unknown frame 'xxx'` |
| `header.stamp` 为 0 | `header.stamp is zero` |
| `header.stamp` 比现在早超过 1 s | `target is 2.431 s old` |
| `position` 里有 nan / inf | `position contains a non-finite number` |
| 四元数没填（模长 < 0.5） | `orientation is not a rotation` |
| 点在工作空间盒外 | `point (0.9, 0.0, 0.24) is outside the accepted workspace box` |

工作空间盒默认 `[-0.30, -0.50, -0.02]` 到 `[0.65, 0.50, 0.80]` m，可用参数
`workspace_min` / `workspace_max` 调整。

> 注意：这是**合理性检查**，不是碰撞检查。MoveIt 当前的场景是**空的**，
> 机械臂还不知道桌子、面板、障碍物在哪。上实机前请确认路径是清的。

日志行为：第一条被拒的消息打 `WARN`（避免 5 Hz 刷屏），
之后累计条数在等待目标时定期报告，任务结束时会打印总拒绝数。

## 6. 怎么自己验证（不需要机械臂）

**方式一：用现成工具发**

```bash
ros2 run piper_elevator_manipulation send_press_target.py \
    --point 0.300,0.000,0.240 --direction 1,0,0
```

它会把方向转成四元数发出去，并打印实际用的四元数，可以现场核对：

```text
topic      : /press/target_pose
frame      : world
point      : 0.3000, 0.0000, 0.2400 m
direction  : +1.0000, +0.0000, +0.0000
quaternion : +0.000000, +0.707107, +0.000000, +0.707107  (x, y, z, w)
```

**方式二：看话题上的原始消息**

```bash
ros2 topic echo /press/target_pose
```

**方式三：看任务节点认不认**

```bash
ros2 run piper_elevator_manipulation press_button_demo --ros-args \
    -p target_source:=topic -p target_wait_timeout_s:=10.0
```

正常会看到：

```text
[INFO] Target source: topic '/press/target_pose', accepted frames world, base_link, ...
[INFO] Waiting for a press target on '/press/target_pose' (giving up after 10.0 s)...
[INFO] Accepted target in 'world': press rod tip at (0.3000, 0.0000, 0.2400), press direction (+1.000, +0.000, +0.000).
```

被拒会看到：

```text
[WARN] Rejected a target on '/press/target_pose': unknown frame 'camera_color_optical_frame'; ...
```

## 7. 感知层最小发布示例

完整的可运行版本在 `scripts/send_press_target.py`。
下面是自包含的最小实现，可以直接抄：

```python
#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def normalise(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if n < 1.0e-9:
        raise ValueError("direction vector is zero")
    return [c / n for c in v]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def quaternion_from_z_axis(direction):
    """返回四元数 (x, y, z, w)，使其 +Z 轴等于 direction。"""
    d = normalise(direction)
    helper = [1.0, 0.0, 0.0] if abs(d[0]) < 0.9 else [0.0, 1.0, 0.0]
    x_axis = normalise(cross(helper, d))
    y_axis = cross(d, x_axis)

    # 旋转矩阵按列摆：第三列就是 d
    m = [
        [x_axis[0], y_axis[0], d[0]],
        [x_axis[1], y_axis[1], d[1]],
        [x_axis[2], y_axis[2], d[2]],
    ]

    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x = 0.25 * s, (m[2][1] - m[1][2]) / s
        y, z = (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w, x = (m[2][1] - m[1][2]) / s, 0.25 * s
        y, z = (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w, x = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s
        y, z = 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w, x = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s
        y, z = (m[1][2] + m[2][1]) / s, 0.25 * s
    return (x, y, z, w)


class ButtonTargetPublisher(Node):
    """把检测到的按钮位置发布成按压目标。"""

    def __init__(self):
        super().__init__("button_target_publisher")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            PoseStamped, "/press/target_pose", qos
        )

    def publish_target(self, position, direction, frame_id="base_link"):
        """position: (x, y, z) 米；direction: 指向按钮内部，不用归一化。"""
        qx, qy, qz, qw = quaternion_from_z_axis(direction)

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()  # ← 别忘了
        message.header.frame_id = frame_id
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = ButtonTargetPublisher()
    # 举例：按钮表面在 (0.30, 0.00, 0.24)，从前方水平压过去
    node.publish_target((0.300, 0.000, 0.240), (1.0, 0.0, 0.0))
    node.get_logger().info("press target published")
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 8. 任务侧怎么开（联调用）

```bash
# 实机：每个目标按一次，然后退出
ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
    target_source:=topic \
    execute:=true hardware_enable:=true

# 实机：一直跑，来一条按一次
ros2 launch piper_elevator_manipulation press_button_hardware.launch.py \
    target_source:=topic max_presses:=0 \
    execute:=true hardware_enable:=true

# 仿真干跑（只规划，不动）
ros2 launch piper_elevator_manipulation press_button_demo.launch.py \
    target_source:=topic
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `target_source` | `parameters` | `topic` = 从话题取目标；`parameters` = 用 `button_x/y/z` 固定点 |
| `target_topic` | `/press/target_pose` | 订阅的话题名 |
| `max_presses` | `1` | 按几次后退出；`0` = 一直跑 |
| `target_wait_timeout_s` | `0` | 等不到有效目标的超时秒数；`0` = 一直等 |
| `target_max_age_s` | `1.0` | 目标最大允许陈旧时间（秒） |
| `press_direction_sign` | `+1` | 设成 `-1` 可把方向反号 |
| `workspace_min` | `[-0.30, -0.50, -0.02]` | 工作空间盒下界（米） |
| `workspace_max` | `[0.65, 0.50, 0.80]` | 工作空间盒上界（米） |
| `approach_distance` | `0.030` | 对准点离按钮表面的距离（米） |
| `press_depth` | `0.003` | 按压前进距离（米） |

实机还需要 `execute:=true` 和 `hardware_enable:=true` **两个**开关同时为真。

## 9. 常见错误清单

1. **四元数没填**，`orientation` 全是 0 → 拒收。
2. **方向取反了**（发的是外法线）→ 机械臂往反方向走。见第 3 节。
3. **`header.stamp` 没填**，或填了 0 → 拒收。C++ 里尤其容易漏。
4. **发到了别的话题**，比如旧的 `/perception/selected_button_pose` →
   任务节点收不到。要改的话用 `target_topic:=` 指定，或者在 launch 里 remap。
5. **`frame_id` 填了相机系** → 拒收，目前没有 TF 变换。
6. **以为要发完整位姿** → 姿态只用来取方向，绕杆自转随便填。
7. **在按压过程中发目标** → 会被丢弃。每次按压都要求一条**新的**观测，
   所以要么持续发，要么等上一次动作结束后再发。

## 10. 目前还没做的（别指望）

- 除 `world` / `base_link` 外的坐标系变换（需要接 TF）
- 按压过程中的接触力反馈（现在是无传感器，按固定行程 3 mm）
- 失败自动重试（现在一次失败就退出，需要上位机重新发目标）
- 碰撞场景（MoveIt 场景是空的，机械臂看不到障碍物）
