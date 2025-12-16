# 四算法动态物体检测跟踪评估系统

## ✨ 系统完成状态

✅ **所有核心功能已实现并测试通过**

## 📁 文件结构

```
evaluation/
├── CMakeLists.txt                    # Catkin构建配置
├── package.xml                       # ROS包元数据
├── README.md                         # 详细技术文档
├── QUICKSTART.md                     # 快速开始指南 ⭐
├── start_evaluation.sh               # 一键启动脚本
│
├── config/
│   └── eval_config.yaml              # 评估参数配置
│
├── launch/
│   └── evaluation_gazebo.launch      # ROS启动文件
│
├── rviz/
│   └── evaluation.rviz               # RViz可视化配置
│
├── scripts/
│   ├── detection_adapter.py          # 数据适配器 (250行)
│   ├── data_association.py           # 匈牙利算法+指标计算 (280行)
│   ├── plot_generator.py             # 图表生成器 (260行)
│   ├── multi_algo_evaluator.py       # 主评估节点 (550行)
│   └── test_system.py                # 系统测试脚本
│
└── results/                          # 输出目录(自动创建)
    ├── frame_metrics.csv             # 逐帧指标
    ├── summary.json                  # 总体报告
    ├── time_series.png/pdf           # 时序曲线图
    ├── radar_chart.png/pdf           # 雷达图
    └── performance_table.png/pdf/txt # 性能表格
```

## 🎯 核心功能

### 1. 统一数据适配器 (`detection_adapter.py`)
- ✅ M-detector点云 → 标准格式 (DBSCAN聚类)
- ✅ FAPP ObjectsStates → 标准格式
- ✅ LV-DOT MarkerArray → 标准格式
- ✅ 通用MarkerArray → 标准格式 (第四算法)

### 2. 数据关联引擎 (`data_association.py`)
- ✅ 匈牙利算法最优匹配
- ✅ 代价矩阵计算 (距离 + 3D IoU)
- ✅ 可配置的距离和IoU阈值
- ✅ TP/FP/FN统计
- ✅ MOTP计算 (平均定位误差)

### 3. 评估指标计算
- ✅ **Recall** (召回率): TP / (TP + FN)
- ✅ **Precision** (精准率): TP / (TP + FP)
- ✅ **F1-Score**: 调和平均数
- ✅ **MOTP**: 匹配物体的平均位置误差
- ✅ 逐帧指标 + 总体统计 (均值/标准差)

### 4. 图表生成器 (`plot_generator.py`)
- ✅ **时序对比曲线图** (2×2子图布局)
  - Recall vs Time
  - Precision vs Time
  - F1-Score vs Time
  - MOTP vs Time
- ✅ **性能雷达图** (4指标维度极坐标)
- ✅ **性能汇总表格** (可视化+文本格式)
- ✅ 自动配置中文字体
- ✅ seaborn美化样式
- ✅ 四算法颜色映射一致性

### 5. 主评估节点 (`multi_algo_evaluator.py`)
- ✅ ROS节点集成
- ✅ 多话题订阅 (真值 + 4个算法)
- ✅ 实时评估定时器 (可配置频率)
- ✅ 数据缓冲与时间同步
- ✅ RViz实时可视化发布
- ✅ 优雅退出与结果保存
- ✅ 详细日志输出

### 6. 可视化系统
- ✅ 真值边界框 (绿色半透明)
- ✅ 各算法检测结果 (不同颜色)
- ✅ 匹配连线显示
- ✅ 实时指标文本叠加
- ✅ RViz配置文件完整

### 7. Gazebo真值采集
- ✅ 订阅 `/gazebo/model_states`
- ✅ 关键词过滤动态物体
- ✅ 自动提取位置、速度
- ✅ 边界框尺寸估计

## 🚀 使用流程

### 最简单方式 (推荐)
```bash
cd /home/st/M_detector/src/M-detector/evaluation
./start_evaluation.sh
# 在tmux窗口中手动启动各组件
```

### 标准方式
```bash
# 1. 启动Gazebo + 算法
roslaunch mot_mapping sim_mapping.launch obj_num:=5
roslaunch m_detector detector_avia.launch
roslaunch onboard_detector run_detector_sim.launch

# 2. 启动评估
roslaunch evaluation evaluation_gazebo.launch

# 3. Ctrl+C 停止并生成报告
```

## 📊 输出结果

### 逐帧数据 (`frame_metrics.csv`)
```csv
Timestamp,Algorithm,TP,FP,FN,Recall,Precision,F1,MOTP
1234567.89,M-detector,5,1,0,1.000,0.833,0.909,0.123
1234567.89,FAPP,4,0,1,0.800,1.000,0.889,0.098
...
```

### 总体报告 (`summary.json`)
```json
{
  "evaluation_time": 1234567890.123,
  "duration": 45.67,
  "parameters": {
    "distance_threshold": 2.0,
    "iou_threshold": 0.1
  },
  "algorithms": {
    "M-detector": {
      "recall_mean": 0.895,
      "recall_std": 0.042,
      "precision_mean": 0.912,
      "precision_std": 0.038,
      "f1_mean": 0.903,
      "f1_std": 0.035,
      "motp_mean": 0.156,
      "motp_std": 0.067,
      "total_tp": 234,
      "total_fp": 23,
      "total_fn": 28
    },
    ...
  }
}
```

### 图表输出
- `time_series.png/pdf`: 高分辨率时序曲线 (14×10英寸, 300 DPI)
- `radar_chart.png/pdf`: 性能雷达图 (10×10英寸)
- `performance_table.png/pdf/txt`: 多格式表格

## 🔧 配置参数

### 核心参数 (`config/eval_config.yaml`)
```yaml
distance_threshold: 2.0      # 匹配距离阈值
iou_threshold: 0.1           # IoU阈值
update_freq: 10.0            # 评估频率
enable_visualization: true   # 可视化开关

# 话题映射
mdetector_topic: "/dynamic_points"
fapp_topic: "/states"
lvdot_topic: "/onboard_detector/dynamic_bboxes"
algo4_topic: "/algo4/detections"

# Gazebo过滤
dynamic_keywords: ["actor", "dynamic", "person", "obstacle"]
default_bbox_size: [0.5, 0.5, 1.5]

# M-detector聚类
mdetector_dbscan_eps: 0.5
mdetector_dbscan_min_samples: 10
```

## ✅ 测试验证

运行测试脚本:
```bash
cd /home/st/M_detector/src/M-detector/evaluation/scripts
python3 test_system.py
```

**预期输出:**
```
============================================================
✅ All tests passed!
============================================================
The evaluation system is ready to use.
```

## 📚 文档资源

1. **QUICKSTART.md** - 5分钟快速上手 ⭐
2. **README.md** - 完整技术文档
3. **本文件** - 系统概览

## 🎨 算法颜色映射

| 算法 | 颜色 | 十六进制 |
|------|------|---------|
| M-detector | 🔴 红色 | #E74C3C |
| FAPP | 🔵 蓝色 | #3498DB |
| LV-DOT | 🟢 绿色 | #2ECC71 |
| Algorithm4 | 🟠 橙色 | #F39C12 |
| Ground Truth | 💚 绿色 | 半透明 |

## 🔬 技术特性

- **模块化设计**: 适配器、关联、计算、可视化分离
- **高效匹配**: scipy优化的匈牙利算法 O(n³)
- **鲁棒性**: 处理缺失数据、异常值、空检测
- **可扩展**: 易于添加新算法和新指标
- **专业级图表**: 出版质量的可视化输出
- **ROS集成**: 原生ROS消息和话题
- **实时+离线**: 支持两种评估模式

## 📈 性能指标

- **代码行数**: ~1500行 (不含注释)
- **依赖**: numpy, scipy, matplotlib, scikit-learn, ROS
- **实时性能**: 10Hz评估频率 (可配置)
- **内存占用**: 缓冲区最大100帧
- **输出格式**: PNG, PDF, CSV, JSON, TXT

## 🚦 下一步建议

1. ✅ **系统已完成** - 可以直接使用
2. 📝 **根据实际调整** `config/eval_config.yaml` 中的话题名称
3. 🎯 **确认第四算法** 的输出格式和话题
4. 🧪 **运行测试场景** 验证各算法性能
5. 📊 **分析结果** 并调优算法参数

## 💡 提示

- 首次使用请阅读 **QUICKSTART.md**
- 遇到问题查看 **README.md** 的常见问题部分
- 运行 `test_system.py` 验证环境
- 使用 `start_evaluation.sh` 快速启动所有组件
- 评估至少运行30秒以获得稳定统计

---

**状态**: ✅ 已完成并测试  
**日期**: 2025-12-16  
**版本**: 1.0.0
