# Piper colored-button perception

> **Status: paused / optional (2026-09-22).** Planning and control development
> now consumes an explicit world-frame target and does not depend on this
> package. The source, tests, dataset tools, and experiment records are retained
> as a reproducible prototype. This package is not started by the default
> simulation or manipulation launch files.

This package owns the perception side of the elevator-button demo.  Its first
tool generates a COCO-format synthetic dataset from the wrist RGB camera.

Generate a 20-image smoke-test dataset:

```bash
ros2 launch piper_button_perception generate_dataset.launch.py \
  output_dir:=/ws/data/button_dataset_smoke \
  num_samples:=20 \
  overwrite:=true
```

Generate the first 500-image dataset:

```bash
ros2 launch piper_button_perception generate_dataset.launch.py \
  output_dir:=/ws/data/button_dataset_v1 \
  num_samples:=500
```

The launch runs Gazebo without RViz or the Gazebo client, randomizes the panel
pose and image appearance, writes JPEG images plus COCO annotations, creates
70/15/15 train/validation/test splits, includes roughly 10% blank-panel negative
samples, and exits automatically when complete.

Render a contact sheet to inspect the generated boxes:

```bash
ros2 run piper_button_perception preview_button_dataset \
  /ws/data/button_dataset_v1
```

## CPU 神经网络训练

训练使用独立的 `piper-training:cpu` 镜像，基于本机已有的
`piper-dev:humble` 镜像添加隔离的 Python 虚拟环境。模型为 TorchVision
SSDLite320 + MobileNetV3 Large，从官方 COCO 权重初始化，分类头改为三类按钮
加背景；第一轮冻结骨干，仅训练检测头。无需 NVIDIA GPU。

以下指令在**宿主机终端**执行，不是在 ROS 容器内执行：

```bash
cd ~/piper_ws/docker
mkdir -p ~/piper_ws/models
sg docker -c 'docker compose -f docker-compose.training.yml build'
sg docker -c 'docker compose -f docker-compose.training.yml run --name piper_button_train_v1 train'
```

默认最多 30 轮，batch size 8，CPU 线程数 6，容器限 6 CPU/8 GB 内存；
验证集 COCO mAP@0.5:0.95 连续 10 轮不改善则提前结束。
输出目录 `/ws/models/button_ssdlite_v1`（宿主机对应 `~/piper_ws/models/button_ssdlite_v1`）：

- `best.pt`：按验证集 mAP 选择的推理权重。
- `last.pt`：最后一轮推理权重；不包含优化器状态，不是断点续训文件。
- `run_config.json`：超参数、软件版本、类别和数据摘要哈希。
- `history.json`：每轮损失、验证指标和耗时。
- `test_metrics.json`：最佳模型在独立测试集上的指标。
- `test_predictions.json` / `test_preview.jpg`：测试预测和可视化。

同一输出目录非空时会拒绝覆盖。新实验通过显式命令选择新目录：

```bash
sg docker -c 'docker compose -f docker-compose.training.yml run --rm train --dataset /ws/data/button_dataset_v1 --output /ws/models/button_ssdlite_new --epochs 30 --threads 6'
```

第二阶段：从第一阶段权重继续微调，解冻骨干，降低学习率（这是新的训练实验，
优化器重新初始化，不是恢复中断）：

```bash
sg docker -c 'docker compose -f docker-compose.training.yml run --name piper_button_train_v2 train --dataset /ws/data/button_dataset_v1 --output /ws/models/button_ssdlite_v2 --initialize /ws/models/button_ssdlite_v1/best.pt --unfreeze-backbone --epochs 20 --lr 0.001 --threads 6'
```

第二阶段的配置、最佳/最终模型和评估文件同样保存在各自输出目录。

对保存的权重重新做独立测试（结果写到新的评估目录）：

```bash
sg docker -c 'docker compose -f docker-compose.training.yml run --rm train --dataset /ws/data/button_dataset_v1 --output /ws/models/button_ssdlite_v1_recheck --checkpoint /ws/models/button_ssdlite_v1/best.pt --threads 6'
```

可在训练结束后，仅用验证集校准部署置信度阈值：

```bash
sg docker -c 'docker compose -f docker-compose.training.yml run --rm --entrypoint /opt/button-venv/bin/python train -m piper_button_perception.calibrate_detector --dataset /ws/data/button_dataset_v1 --checkpoint /ws/models/button_ssdlite_v1/best.pt --output /ws/models/button_ssdlite_v1/calibrated'
```

默认在 0.05～0.95、步长 0.05 的网格中选取满足每类 precision/recall ≥ 0.98 且负样本零误检的
最高阈值，随后固定该阈值评估测试集。若验证集没有阈值达标，则明确报告失败，
不自动指定部署阈值。校准结果保存到 `calibrated/calibration.json`，原始 0.7
阈值的报告保留不变。测试集不得用于挑选阈值。

启动后台训练并查看日志：

```bash
sg docker -c 'docker compose -f docker-compose.training.yml run -d --name piper_button_train_v1 train'
sg docker -c 'docker logs -f piper_button_train_v1'
```

前台/后台两种启动方式选一种，不要重复运行。已有同名容器时，应先检查
`docker ps -a`，保留现有实验并为新实验指定不同容器名及输出目录。

检查坐标还原、负样本及评估逻辑：

```bash
sg docker -c 'docker compose -f docker-compose.training.yml run --rm --entrypoint /opt/button-venv/bin/python train -m unittest discover -s /ws/src/Piper_ros/src/piper_button_perception/test -v'
```

预处理为 RGB 等比例缩放后补灰边至 320×320；预测框恢复到原始图像坐标。
白框表示真值，彩色框表示达到该次评估阈值的预测。默认报告的 precision/recall
采用置信度 0.7、匹配 IoU 0.5；COCO AP 使用完整低阈值预测独立计算。
单帧耗时仅包含神经网络推理，不包含 ROS 通信、图像读取或三维定位。

当前数据来自同一仿真场景的随机扰动，测试分数只衡量该场景分布内的效果；
它不能证明对新按钮排列、真实电梯、复杂背景的泛化能力。三维定位、ROS 在线
推理和实机数据验证是后续阶段，本次训练不向机械臂发布运动指令。

模型接口参考 [TorchVision SSDLite 文档](https://docs.pytorch.org/vision/0.22/models/generated/torchvision.models.detection.ssdlite320_mobilenet_v3_large.html)。

## 2026-09-22 首次训练结果

已完成第一阶段 30 轮和第二阶段 20 轮。第二阶段按验证集 mAP@0.5:0.95
选择第 10 轮权重，保存在 `/ws/models/button_ssdlite_v2/best.pt`。
75 张测试图像的 mAP@0.5 为 0.9991，mAP@0.5:0.95 为 0.9072；
CPU 单帧模型推理中位耗时约 19.95 ms。

默认 0.7 阈值下，蓝/红/绿召回率分别约 81.8%/100%/62.1%。
粗网格未找到满足验证集条件的阈值；进一步仅在验证集上搜索
`--thresholds 0.11 0.12 0.13 0.14 0.15`，选择 0.12。
该阈值的测试召回率为 97.0%/100%/100%，但 9 张空面板有 2 张误检，
因此**未通过最终验收，不应直接用于稳定下发按钮目标**。
不能将高 mAP 等同于固定阈值下的高召回率与零误检。

详细记录见 `/ws/models/button_ssdlite_v2/model_card.json` 与
`/ws/models/button_ssdlite_v2/calibrated_fine/`。后续应增加独立场景、
按钮布局、部分可见按钮及困难负样本，再通过新的验证/测试场景检验泛化，
不要继续针对本次测试集调参。
