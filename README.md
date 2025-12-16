# Multi-Algorithm Evaluation Package

四算法动态物体检测跟踪对比评估系统

## 功能特性

- ✅ 支持四种算法对比评估: M-detector, FAPP, LV-DOT, 自定义算法
- ✅ 统一数据适配器,处理不同输出格式
- ✅ 基于匈牙利算法的精确数据关联
- ✅ 计算召回率、精准率、F1-score和MOTP指标
- ✅ 生成时序对比曲线图、雷达图和性能表格
- ✅ RViz实时可视化
- ✅ 从Gazebo获取真值数据
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

### 1. 启动Gazebo仿真

```bash
# 使用FAPP的仿真环境
roslaunch mot_mapping sim_mapping.launch obj_num:=5
```

### 2. 启动检测算法

在不同终端中启动各算法:

```bash
# 终端1: M-detector
roslaunch m_detector detector_avia.launch

# 终端2: FAPP (如果使用独立节点)
roslaunch mot_mapping mapping_sim.launch

# 终端3: LV-DOT
roslaunch onboard_detector run_detector_sim.launch

# 终端4: 第四算法 (根据实际情况调整)
# roslaunch your_algo detector.launch
```

### 3. 启动评估系统

```bash
# 启动评估器(包含RViz)
roslaunch evaluation evaluation_gazebo.launch

# 或不启动RViz
roslaunch evaluation evaluation_gazebo.launch rviz:=false
```

### 4. 停止评估并生成报告

按 `Ctrl+C` 停止评估节点,系统会自动:
- 保存逐帧指标到 `results/frame_metrics.csv`
- 生成总体报告 `results/summary.json`
- 绘制时序对比曲线 `results/time_series.png`
- 绘制雷达图 `results/radar_chart.png`
- 生成性能表格 `results/performance_table.png`

## 配置说明

编辑 `config/eval_config.yaml` 修改参数:

```yaml
# 数据关联参数
distance_threshold: 2.0      # 最大匹配距离(米)
iou_threshold: 0.1           # 最小IoU阈值

# 更新频率
update_freq: 10.0            # 评估更新频率(Hz)

# 话题配置 (根据实际话题名称修改)
ground_truth_topic: "/gazebo/model_states"
mdetector_topic: "/dynamic_points"
fapp_topic: "/states"
lvdot_topic: "/onboard_detector/dynamic_bboxes"
algo4_topic: "/algo4/detections"

# Gazebo动态物体过滤关键词
dynamic_keywords: ["actor", "dynamic", "person", "obstacle", "sphere", "box"]
```

## 话题接口

### 订阅话题

| 话题名 | 消息类型 | 说明 |
|--------|---------|------|
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | Gazebo真值 |
| `/m_detector/point_out` | `sensor_msgs/PointCloud2` | M-detector动态点云 |
| `/dynamic_points` | `sensor_msgs/PointCloud2` | FAPP动态点云 |
| `/onboard_detector/dynamic_point_cloud` | `sensor_msgs/PointCloud2` | LV-DOT动态点云 |
| `/algo4/dynamic_points` | `sensor_msgs/PointCloud2` | 第四算法动态点云 |

**注意**: 所有算法统一订阅未聚类的动态点云,评估器使用统一的DBSCAN聚类生成边界框,确保检测框生成机制一致,对比更公平。

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

### Q: 如何添加第四个算法?

A: 修改 `config/eval_config.yaml` 中的 `algo4_topic`,确保第四算法发布 `MarkerArray` 格式的检测结果。

### Q: 某个算法没有检测输出怎么办?

A: 评估器会自动跳过没有数据的算法,不影响其他算法的评估。

### Q: 如何修改M-detector的聚类参数?

A: 编辑 `config/eval_config.yaml`:
```yaml
mdetector_dbscan_eps: 0.5
mdetector_dbscan_min_samples: 10
```

### Q: 如何离线评估rosbag?

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
├── scripts/
│   ├── detection_adapter.py      # 数据适配器
│   ├── data_association.py       # 匈牙利算法和指标计算
│   ├── plot_generator.py         # 图表生成
│   └── multi_algo_evaluator.py   # 主评估节点
├── launch/
│   └── evaluation_gazebo.launch  # 启动文件
├── config/
│   └── eval_config.yaml          # 配置文件
├── rviz/
│   └── evaluation.rviz           # RViz配置
└── results/                      # 输出目录
    ├── frame_metrics.csv
    ├── summary.json
    ├── time_series.png
    ├── radar_chart.png
    └── performance_table.png
```

## 许可证

MIT License

## 联系方式

如有问题请提交Issue或联系维护者。
