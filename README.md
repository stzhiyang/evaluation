# Multi-Algorithm Evaluation Package

四算法动态物体检测跟踪对比评估系统

## 功能特性

- ✅ 支持四种算法对比评估: M-detector, FAPP, LV-DOT, LDOT
- ✅ 统一数据适配器,处理不同输出格式
- ✅ 基于匈牙利算法的精确数据关联
- ✅ 计算召回率、精准率、F1-score和MOTP指标
- ✅ 生成时序对比曲线图、雷达图和性能表格
- ✅ RViz实时可视化
- ✅ **支持Gazebo仿真和动捕真值** ⭐ 新增
- ✅ **动捕模式：XY平面距离关联，ID映射Z轴高度** ⭐ 新增
- ✅ 支持在线和离线评估模式

## 安装依赖

```bash
# Python依赖
pip3 install numpy scipy matplotlib scikit-learn

# ROS依赖 (在工作空间根目录)
cd /home/st/M_detector
catkin_make
source devel/setup.bash
```

## 使用方法

### Gazebo仿真评估

#### 1. 启动Gazebo仿真

```bash
# 使用FAPP的仿真环境
roslaunch mot_mapping sim_mapping.launch obj_num:=5
```

#### 2. 启动检测算法

在不同终端中启动各算法:

```bash
# 终端1: M-detector
roslaunch m_detector detector_avia.launch

# 终端2: FAPP (如果使用独立节点)
roslaunch mot_mapping mapping_sim.launch

# 终端3: LV-DOT
roslaunch onboard_detector run_detector_sim.launch

# 终端4: LDOT
roslaunch ldot_detector detector.launch
```

#### 3. 启动评估系统

```bash
# 启动评估器(包含RViz)
roslaunch evaluation evaluation_gazebo.launch

# 或不启动RViz
roslaunch evaluation evaluation_gazebo.launch rviz:=false
```

### 动捕真值评估 ⭐ 新增

#### 1. 配置动捕参数

编辑 `config/eval_config_mocap.yaml`：

```yaml
# 设置动捕话题
ground_truth_topic: "/vrpn_client_node/poses"

# 配置物体ID到Z轴高度的映射
mocap_id_height_map:
  car: 0.5        # 车辆中心高度
  person: 1.0     # 人体中心高度
  drone: 1.5      # 无人机中心高度

# 设置物体类别顺序（必须与动捕消息中poses顺序一致）
mocap_object_classes: ['car', 'person', 'drone']
```

#### 2. 启动动捕系统

```bash
# 启动动捕客户端（以VRPN为例）
roslaunch vrpn_client_ros sample.launch
```

#### 3. 启动评估系统

```bash
# 启动动捕评估器
roslaunch evaluation evaluation_mocap.launch

# 或使用RViz可视化
roslaunch evaluation evaluation_mocap.launch rviz:=true
```

**详细说明**：请参考 [`docs/MOCAP_USAGE.md`](docs/MOCAP_USAGE.md)

### 通用步骤

#### 4. 停止评估并生成报告

按 `Ctrl+C` 停止评估节点,系统会自动:
- 保存逐帧指标到 `results/frame_metrics.csv`
- 生成总体报告 `results/summary.json`
- 绘制时序对比曲线 `results/time_series.png`
- 绘制雷达图 `results/radar_chart.png`
- 生成性能表格 `results/performance_table.png`

## 配置说明

### Gazebo配置 (`config/eval_config.yaml`)

```yaml
# 真值来源
ground_truth_source: "gazebo"
ground_truth_topic: "/gazebo/model_states"

# 数据关联参数
distance_threshold: 2.0      # 3D欧氏距离阈值(米)
iou_threshold: 0.1           # 最小IoU阈值

# Gazebo动态物体过滤关键词
dynamic_keywords: ["actor", "dynamic", "person", "obstacle", "sphere", "box"]
```

### 动捕配置 (`config/eval_config_mocap.yaml`) ⭐ 新增

```yaml
# 真值来源
ground_truth_source: "mocap"
ground_truth_topic: "/vrpn_client_node/poses"

# 动捕模式
mocap_mode: true  # 启用XY平面距离关联

# ID到Z轴高度映射
mocap_id_height_map:
  car: 0.5
  person: 1.0
  drone: 1.5

# 物体类别顺序（与动捕消息poses顺序一致）
mocap_object_classes: ['car', 'person', 'drone']

# 数据关联参数
distance_threshold: 2.0      # XY平面距离阈值(米)
iou_threshold: 0.1
```

## 话题接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | Gazebo真值 (gazebo模式) |
| `/vrpn_client_node/poses` | `geometry_msgs/PoseArray` | 动捕真值 (mocap模式) ⭐ |
| `/m_detector/point_out` | `sensor_msgs/PointCloud2` | M-detector动态点云 |
| `/states` | `obj_state_msgs/ObjectsStates` | FAPP动态物体状态 |
| `/onboard_detector/dynamic_bboxes` | `visualization_msgs/MarkerArray` | LV-DOT动态边界框 |
| `/ldot_detector/dynamic_bboxes` | `visualization_msgs/MarkerArray` | LDOT动态边界框 |

### 发布话题 (可视化)

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/evaluation/ground_truth` | `visualization_msgs/MarkerArray` | 真值边界框(绿色) |
| `/evaluation/mdetector_detections` | `visualization_msgs/MarkerArray` | M-detector检测(红色) |
| `/evaluation/fapp_detections` | `visualization_msgs/MarkerArray` | FAPP检测(蓝色) |
| `/evaluation/lvdot_detections` | `visualization_msgs/MarkerArray` | LV-DOT检测(绿色) |
| `/evaluation/algo4_detections` | `visualization_msgs/MarkerArray` | 算法4检测(橙色) |
| `/evaluation/match_lines` | `visualization_msgs/Marker` | 匹配连线 |
| `/evaluation/metrics_text` | `visualization_msgs/MarkerArray` | 实时指标文本 |

## 输出文件说明

### frame_metrics.csv
逐帧评估指标,包含每个时间戳的TP/FP/FN和各项指标

### summary.json
总体评估报告,包含:
- 评估时长和参数
- 各算法的均值和标准差
- 总体TP/FP/FN统计

### time_series.png/pdf
时序对比曲线图,包含4个子图:
- Recall vs Time
- Precision vs Time
- F1-Score vs Time
- MOTP vs Time

### radar_chart.png/pdf
性能雷达图,显示四个算法在四个指标上的综合表现

### performance_table.png/pdf/txt
性能汇总表格,以均值±标准差形式展示各算法性能

## 评估指标说明

- **Recall (召回率)**: TP / (TP + FN) - 有多少真实物体被检测到
- **Precision (精准率)**: TP / (TP + FP) - 检测结果中有多少是正确的
- **F1-Score**: 2 × P × R / (P + R) - 综合指标
- **MOTP (定位误差)**: 平均位置误差(米),越小越好

## 常见问题

### Gazebo模式

#### Q: 如何添加第四个算法?

A: 修改 `config/eval_config.yaml` 中对应算法的话题，确保算法发布正确格式的检测结果。

#### Q: 某个算法没有检测输出怎么办?

A: 评估器会自动跳过没有数据的算法,不影响其他算法的评估。

#### Q: 如何修改M-detector的聚类参数?

A: 编辑配置文件:
```yaml
mdetector_dbscan_eps: 0.5
mdetector_dbscan_min_samples: 10
```

### 动捕模式 ⭐ 新增

#### Q: 动捕模式与Gazebo模式有什么区别?

A: 主要区别：
- **真值来源**：动捕PoseArray vs Gazebo ModelStates
- **位置信息**：仅XY平面 vs 完整3D
- **关联方式**：XY平面距离 vs 3D欧氏距离
- **Z轴高度**：配置文件映射 vs 直接获取

#### Q: 如何确定物体类别顺序?

A: 使用以下命令查看动捕消息：
```bash
rostopic echo /vrpn_client_node/poses
```
按照poses数组的顺序设置`mocap_object_classes`。

#### Q: Z轴高度应该设置为多少?

A: 设置为物体中心点的高度（非底部）。例如：
- 人高1.8m → 中心约1.0m
- 车高1.0m → 中心约0.5m
- 无人机飞行高度1.5m → 设置1.5m

#### Q: 关联效果不好怎么办?

A: 调整以下参数：
1. 增大`distance_threshold`（允许更大的XY距离误差）
2. 降低`iou_threshold`（放宽边界框IOU要求）
3. 检查物体类别顺序是否正确

### 通用问题

#### Q: 如何离线评估rosbag?

A: 
```bash
# 录制必要话题
rosbag record /gazebo/model_states /dynamic_points /states /onboard_detector/dynamic_bboxes -O test.bag

# 重放并评估
roslaunch evaluation evaluation_gazebo.launch &
rosbag play test.bag
# Ctrl+C停止评估并生成报告
```

## 文件结构

```
evaluation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── docs/
│   └── MOCAP_USAGE.md            # 动捕使用详细说明 ⭐
├── scripts/
│   ├── detection_adapter.py      # 数据适配器（含动捕支持）⭐
│   ├── data_association.py       # 匈牙利算法（含XY模式）⭐
│   ├── plot_generator.py         # 图表生成
│   └── multi_algo_evaluator.py   # 主评估节点（含动捕模式）⭐
├── launch/
│   ├── evaluation_gazebo.launch  # Gazebo评估
│   ├── evaluation_gazebo_sim.launch
│   ├── evaluation_mocap.launch   # 动捕评估 ⭐
│   ├── performance_evaluation.launch
│   └── timing_calculation.launch
├── config/
│   ├── eval_config.yaml          # Gazebo配置
│   ├── eval_config_sim.yaml
│   ├── eval_config_mocap.yaml    # 动捕配置 ⭐
│   └── performance_config.yaml
├── rviz/
│   └── evaluation.rviz           # RViz配置
└── results/                      # 输出目录
    ├── algorithm_eval/
    ├── performance_eval/
    └── timing_eval/
```

## 许可证

MIT License

## 联系方式

如有问题请提交Issue或联系维护者。
