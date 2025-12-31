# 动捕真值评估系统使用说明

## 概述

本系统支持使用动捕（Motion Capture）系统提供的真值数据来评估动态物体检测算法的性能。与Gazebo仿真不同，动捕系统只提供物体的XY平面位置信息，因此系统做了如下适配：

### 主要特点

1. **真值来源**：动捕系统发布的`geometry_msgs/PoseArray`消息
2. **关联方式**：只使用XY平面距离进行数据关联，忽略Z轴差异
3. **Z轴高度**：通过配置文件的ID映射直接赋值
4. **支持类别**：车辆、人、无人机（可扩展）

## 配置文件

### 1. 动捕模式配置 (`config/eval_config_mocap.yaml`)

关键参数说明：

```yaml
# 真值数据源
ground_truth_source: "mocap"  # 使用动捕作为真值源
ground_truth_topic: "/vrpn_client_node/poses"  # 动捕真值话题（根据实际情况修改）

# 动捕模式
mocap_mode: true  # 启用XY平面距离关联

# ID到Z轴高度的映射
mocap_id_height_map:
  car: 0.5        # 车辆中心高度（米）
  person: 1.0     # 人体中心高度（米）
  drone: 1.5      # 无人机中心高度（米）

# 物体类别列表（与动捕消息中poses的顺序对应）
mocap_object_classes: ['car', 'person', 'drone']
```

**重要提示**：`mocap_object_classes`列表的顺序必须与动捕系统发布的PoseArray中poses的顺序完全一致！

### 2. 修改配置以适应实际环境

根据您的实际动捕环境，需要修改以下内容：

#### a. 动捕话题名称
```yaml
ground_truth_topic: "/your/mocap/topic"  # 修改为实际的动捕话题
```

#### b. 物体ID和高度映射
根据实际物体的中心高度设置：
```yaml
mocap_id_height_map:
  car: 0.5        # 车辆：如果车辆高度1.0m，中心约0.5m
  person: 1.0     # 人：如果人高1.8m，中心约1.0m
  drone: 1.5      # 无人机：飞行高度约1.5m
```

#### c. 物体类别顺序
假设动捕系统按顺序追踪：第1个目标是车、第2个是人、第3个是无人机
```yaml
mocap_object_classes: ['car', 'person', 'drone']
```

如果顺序不同，例如：人、无人机、车，则修改为：
```yaml
mocap_object_classes: ['person', 'drone', 'car']
```

#### d. 匹配阈值
根据实际场景调整：
```yaml
# 数据关联参数
distance_threshold: 2.0  # XY平面距离阈值（米）
iou_threshold: 0.1       # 不使用（动捕模式不需要）
```

## 使用方法

### 1. 启动评估系统

```bash
roslaunch evaluation evaluation_mocap.launch
```

可选参数：
- 启用RViz可视化：
  ```bash
  roslaunch evaluation evaluation_mocap.launch rviz:=true
  ```

- 使用自定义配置文件：
  ```bash
  roslaunch evaluation evaluation_mocap.launch config_file:=/path/to/your/config.yaml
  ```

### 2. 检查系统状态

启动后查看终端输出，应该看到：

```
[INFO] Ground truth source: mocap
[INFO] Mocap mode enabled: XY plane distance matching only
[INFO] Mocap ID-height mapping: {'car': 0.5, 'person': 1.0, 'drone': 1.5}
[INFO] Mocap object classes: ['car', 'person', 'drone']
[INFO] Matching criteria (both must be satisfied):
[INFO]   - 3D Bounding Box IOU >= 0.1
[INFO]   - XY plane distance < 2.0m (mocap mode)
```

### 3. 查看评估结果

结果保存在：`results/algorithm_eval/eval_YYYYMMDD_HHMMSS/`

包含文件：
- `frame_metrics_*.csv`：逐帧评估指标
- `performance_table_*.txt`：性能汇总表格
- `summary_*.json`：JSON格式的评估摘要

## 工作原理

### 1. 数据流程

```
动捕系统 → PoseArray消息 → mocap_gt_callback()
   ↓
提取XY位置 + 从配置获取Z高度 → StandardDetection格式
   ↓
数据关联（XY平面距离 + 2D IOU）
   ↓
评估指标计算
```

### 2. 关联算法

- **Gazebo模式**：
  - 距离：3D欧氏距离 `sqrt(dx² + dy² + dz²)`
  - IOU：3D边界框IOU（体积交并比）
  - 条件：两者必须同时满足

- **动捕模式**：
  - 距离：XY平面距离 `sqrt(dx² + dy²)`
  - IOU：不使用
  - 条件：只看距离

这样可以避免Z轴测量误差或飞行高度变化对关联准确性的影响。**动捕只提供位置信息，因此关联只看距离**。

### 3. Z轴高度处理

由于动捕只提供XY位置，系统通过以下方式确定Z高度：

1. 从`mocap_object_classes`获取物体类别
2. 在`mocap_id_height_map`中查找对应的Z高度
3. 组合成3D位置用于可视化和边界框计算

## 注意事项

### 1. 消息格式要求

动捕系统必须发布`geometry_msgs/PoseArray`消息，格式为：
```
header:
  stamp: ...
  frame_id: "world"
poses:
  - position: {x: 1.0, y: 2.0, z: 0.0}  # z值会被忽略
    orientation: ...
  - position: {x: 3.0, y: 4.0, z: 0.0}
    orientation: ...
```

### 2. 类别顺序匹配

**最重要**：确保`mocap_object_classes`的顺序与动捕消息中poses的顺序完全匹配！

错误示例：
- 动捕发布顺序：[车, 人, 无人机]
- 配置顺序：['person', 'car', 'drone'] ❌ 错误！

正确示例：
- 动捕发布顺序：[车, 人, 无人机]
- 配置顺序：['car', 'person', 'drone'] ✓ 正确

### 3. 坐标系对齐

确保动捕坐标系与算法输出坐标系一致。如果不一致，可能需要添加坐标变换。

### 4. 默认边界框尺寸

系统根据物体类别自动设置边界框尺寸：
- 车辆：[1.5, 1.0, 0.8] 米
- 人：[0.5, 0.5, 1.8] 米
- 无人机：[0.6, 0.6, 0.3] 米

如需修改，可在`detection_adapter.py`的`parse_mocap()`函数中调整。

## 示例场景

### 场景1：室内多目标跟踪

环境：室内有1辆小车、2个人、1架无人机

配置：
```yaml
mocap_id_height_map:
  car: 0.3
  person: 1.0
  drone: 1.2
mocap_object_classes: ['car', 'person', 'person', 'drone']
```

### 场景2：室外无人机集群

环境：3架无人机在不同高度飞行

配置：
```yaml
mocap_id_height_map:
  drone_low: 1.0
  drone_mid: 1.5
  drone_high: 2.0
mocap_object_classes: ['drone_low', 'drone_mid', 'drone_high']
```

## 故障排查

### 问题1：无法接收到真值数据

检查：
1. 动捕系统是否正在运行
2. 话题名称是否正确：`rostopic list | grep mocap`
3. 消息类型是否为PoseArray：`rostopic info /your/mocap/topic`

### 问题2：关联效果不好

调整：
1. 增大`distance_threshold`
2. 降低`iou_threshold`
3. 检查物体类别顺序是否正确

### 问题3：Z轴高度显示异常

检查：
1. `mocap_id_height_map`是否包含所有类别
2. 高度值是否合理（中心位置，非底部）
## 与Gazebo模式的对比

| 特性 | Gazebo模式 | 动捕模式 |
|------|-----------|------|
| 真值来源 | ModelStates消息 | PoseArray消息 |
| 位置信息 | 完敤3D位置+姿态 | 仅XY平面位置 |
| 速度信息 | 提供 | 不提供 |
| Z轴高度 | 直接获取 | 配置文件映射 |
| 距离计算 | 3D欧氏距离 | XY平面距离 |
| IOU计算 | 3D边界框IOU | 不使用 |
| 匹配条件 | 距离+IOU双重 | 只看距离 |
| 适用场景 | 仿真实验 | 真实环境测试 |
| 适用场景 | 仿真实验 | 真实环境测试 |

## 更多信息

- 源代码：`scripts/multi_algo_evaluator.py`
- 适配器：`scripts/detection_adapter.py`
- 关联算法：`scripts/data_association.py`
