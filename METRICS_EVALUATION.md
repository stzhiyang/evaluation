# 算法性能指标评估功能

## 概述

算法性能指标评估功能用于评估动态物体检测算法的准确性和可靠性。通过将算法检测结果与 Gazebo 仿真环境中的真实物体位置（Ground Truth）进行对比，计算多个关键性能指标，全面评估算法的检测性能。

## 功能特点

### 1. 多算法同时评估
- 支持同时评估 4 个算法：M-detector、FAPP、LV-DOT、LDOT
- 统一的评估标准和数据关联方法
- 自动生成多算法对比报告

### 2. 全面的性能指标
- **检测准确性**：Precision（精确率）、Recall（召回率）、F1-Score
- **位置精度**：平均位置误差（ALE）
- **实时性能**：检测延迟、处理频率
- **稳定性**：指标标准差、波动范围

### 3. 智能数据关联
- 基于距离的匹配算法
- 自动处理一对多、多对一情况
- 支持可配置的匹配阈值

### 4. 丰富的输出格式
- **CSV 数据文件**：逐帧指标记录，便于后续分析
- **可视化图表**：自动生成性能曲线和对比图
- **文本报告**：汇总统计结果和算法排名
- **RViz 可视化**：实时显示检测结果和真值对比

## 核心概念

### 性能指标定义

#### 1. Precision（精确率）
```
Precision = TP / (TP + FP)
```

**含义**：算法检测出的物体中，有多少是真实存在的

**示例：**
- 算法检测到 10 个物体
- 其中 8 个是真实物体（TP = 8）
- 2 个是误检测（FP = 2）
- Precision = 8 / (8 + 2) = 0.8 = 80%

**高 Precision 的意义**：
- 误报率低
- 检测结果可信度高
- 适合对误报敏感的应用

#### 2. Recall（召回率）
```
Recall = TP / (TP + FN)
```

**含义**：真实存在的物体中，有多少被算法检测到

**示例：**
- 场景中有 10 个真实物体
- 算法检测到 8 个（TP = 8）
- 漏检 2 个（FN = 2）
- Recall = 8 / (8 + 2) = 0.8 = 80%

**高 Recall 的意义**：
- 漏检率低
- 检测覆盖率高
- 适合对漏检敏感的应用

#### 3. F1-Score（综合指标）
```
F1 = 2 × (Precision × Recall) / (Precision + Recall)
```

**含义**：精确率和召回率的调和平均数

**示例：**
- Precision = 0.8
- Recall = 0.9
- F1 = 2 × (0.8 × 0.9) / (0.8 + 0.9) = 0.847

**F1-Score 的意义**：
- 平衡精确率和召回率
- 综合评估算法性能
- 常用于算法对比

#### 4. ALE（平均位置误差）
```
ALE = Σ distance(detected, ground_truth) / N
```

**含义**：检测位置与真实位置的平均距离误差

**示例：**
- 检测到 3 个物体
- 误差分别为：0.2m, 0.3m, 0.15m
- ALE = (0.2 + 0.3 + 0.15) / 3 = 0.217m

**低 ALE 的意义**：
- 定位精度高
- 适合需要精确位置的应用

### 数据关联方法

#### 匹配规则

1. **距离阈值**：检测点与真值点的距离 < 阈值（默认 2.0m）
2. **最近邻匹配**：优先匹配距离最近的点对
3. **一对一匹配**：每个真值点最多匹配一个检测点

#### 匹配结果分类

```
True Positive (TP):  检测点成功匹配到真值点
False Positive (FP): 检测点未匹配到任何真值点（误检）
False Negative (FN): 真值点未被任何检测点匹配（漏检）
```

#### 匹配示例

**场景：**
- 真值点：A(0,0), B(5,0), C(10,0)
- 检测点：D1(0.3,0), D2(5.2,0), D3(15,0), D4(20,0)
- 距离阈值：2.0m

**匹配过程：**
1. D1 与 A 距离 0.3m < 2.0m → 匹配成功（TP）
2. D2 与 B 距离 0.2m < 2.0m → 匹配成功（TP）
3. D3 与 C 距离 5.0m > 2.0m → 未匹配（FP）
4. D4 距离所有真值点都 > 2.0m → 未匹配（FP）
5. C 未被匹配 → 漏检（FN）

**结果：**
- TP = 2, FP = 2, FN = 1
- Precision = 2/(2+2) = 0.5
- Recall = 2/(2+1) = 0.67
- F1 = 0.57

## 文件结构

```
evaluation/
├── launch/
│   └── evaluation_gazebo.launch         # 评估启动文件
├── config/
│   └── eval_config.yaml                 # 评估配置文件
├── scripts/
│   ├── multi_algo_evaluator.py          # 主评估脚本
│   ├── detection_adapter.py             # 检测数据适配器
│   ├── data_association.py              # 数据关联和指标计算
│   └── plot_generator.py                # 图表生成器
└── results/
    └── eval_<时间戳>/                   # 结果目录
        ├── M-detector_metrics.csv       # M-detector 指标数据
        ├── FAPP_metrics.csv             # FAPP 指标数据
        ├── LV-DOT_metrics.csv           # LV-DOT 指标数据
        ├── LDOT_metrics.csv             # LDOT 指标数据
        ├── summary_report.txt           # 汇总报告
        └── plots/                       # 可视化图表
            ├── precision_comparison.png
            ├── recall_comparison.png
            ├── f1_comparison.png
            └── ale_comparison.png
```

## 使用方法

### 步骤 1：配置评估参数

编辑 `evaluation/config/eval_config.yaml`：

```yaml
# 数据关联参数
distance_threshold: 2.0      # 最大匹配距离(米)

# 更新频率
update_freq: 10.0            # 评估更新频率(Hz)

# 评估控制参数
max_frames: 300              # 最大评估帧数，0表示无限制

# 输出目录
output_dir: "results"

# 话题配置
ground_truth_topic: "/gazebo/model_states"
mdetector_topic: "/m_detector/point_out"
fapp_topic: "/dynamic_points"
lvdot_topic: "/onboard_detector/dynamic_point_cloud"
ldot_topic: "/ldot_detector/dynamic_point_cloud"

# Gazebo 动态物体过滤关键词
dynamic_keywords: ["person", "actor", "dynamic"]

# 默认边界框尺寸 [长, 宽, 高] (米)
default_bbox_size: [0.5, 0.5, 1.8]

# DBSCAN 聚类参数
mdetector_dbscan_eps: 0.5
mdetector_dbscan_min_samples: 10

# 质心补偿配置
enable_centroid_compensation: true

# 里程计话题
odom_topic: "/Odometry"

# 激光雷达外参 [x, y, z, roll, pitch, yaw]
lidar_extrinsics: [-0.005, 0.0, 0.06, 0.0, 0.0, 0.0]
```

### 步骤 2：启动 Gazebo 仿真

确保 Gazebo 仿真环境已启动，并包含动态物体：

```bash
roslaunch your_simulation simulation.launch
```

### 步骤 3：启动算法节点

启动需要评估的算法（可以启动 1-4 个）：

```bash
# M-detector
roslaunch m_detector m_detector.launch

# FAPP
roslaunch fapp fapp.launch

# LV-DOT
roslaunch lv_dot lv_dot.launch

# LDOT
roslaunch ldot ldot.launch
```

### 步骤 4：启动评估器

```bash
roslaunch evaluation evaluation_gazebo.launch
```

### 步骤 5：观察评估过程

评估器会在终端输出实时信息：

```
Multi-Algorithm Evaluator initialized
Algorithms: M-detector, FAPP, LV-DOT, LDOT
Update frequency: 10.0 Hz
Max evaluation frames: 300

Frame 1/300:
  M-detector: P=0.85, R=0.90, F1=0.87, ALE=0.23m
  FAPP:       P=0.78, R=0.85, F1=0.81, ALE=0.31m
  LV-DOT:     P=0.92, R=0.88, F1=0.90, ALE=0.18m
  LDOT:       P=0.88, R=0.92, F1=0.90, ALE=0.21m
```

### 步骤 6：查看结果

评估完成后，结果自动保存到 `evaluation/results/eval_<时间戳>/`

## 配置参数详解

### 距离阈值 (distance_threshold)

**作用**：判断检测点和真值点是否匹配的最大距离

**选择建议：**

| 阈值 | 适用场景 | 优点 | 缺点 |
|-----|---------|------|------|
| 0.5m | 高精度定位 | 严格匹配 | 可能漏检 |
| 1.0m | 一般应用 | 平衡性能 | - |
| 2.0m | 宽松匹配 | 容错性好 | 可能误匹配 |
| 3.0m+ | 粗略检测 | 召回率高 | 精度要求低 |

**推荐值**：2.0m（适合大多数场景）

### 更新频率 (update_freq)

**作用**：评估器计算指标的频率

**选择建议：**
- 5 Hz：快速响应，适合实时监控
- 10 Hz：**推荐值**，平衡性能和精度
- 20 Hz：高频率，系统开销大

### 最大评估帧数 (max_frames)

**作用**：限制评估的总帧数，用于公平对比

**使用场景：**
- 0：无限制，持续评估直到手动停止
- 300：评估 300 帧后自动停止（约 30 秒 @ 10Hz）
- 600：评估 600 帧后自动停止（约 60 秒 @ 10Hz）

**建议：**
- 开发调试：0（无限制）
- 性能对比：300-600（固定帧数）
- 论文数据：1000+（充分统计）

### 动态物体关键词 (dynamic_keywords)

**作用**：从 Gazebo 模型中筛选动态物体

**示例：**
```yaml
dynamic_keywords: ["person", "actor", "dynamic", "obstacle"]
```

**匹配规则**：模型名称包含任一关键词即被识别为动态物体

**常见模型名称：**
- `actor_0`, `actor_1` → 使用 "actor"
- `person_walking` → 使用 "person"
- `dynamic_box_1` → 使用 "dynamic"

## 输出文件说明

### CSV 指标文件

每个算法生成一个 CSV 文件，记录逐帧指标：

**文件名**：`<算法名>_metrics.csv`

**列定义：**
```csv
timestamp,precision,recall,f1_score,ale,tp,fp,fn,gt_count,det_count
1702987234.123,0.85,0.90,0.87,0.23,9,2,1,10,11
1702987234.223,0.88,0.92,0.90,0.21,11,1,1,12,12
...
```

**字段说明：**
- `timestamp`：时间戳（秒）
- `precision`：精确率（0-1）
- `recall`：召回率（0-1）
- `f1_score`：F1 分数（0-1）
- `ale`：平均位置误差（米）
- `tp`：真阳性数量
- `fp`：假阳性数量
- `fn`：假阴性数量
- `gt_count`：真值物体数量
- `det_count`：检测物体数量

### 汇总报告文件

**文件名**：`summary_report.txt`

**内容示例：**
```
==================================================
多算法性能评估报告
==================================================

评估时间: 2025-12-19 16:30:45
评估帧数: 300
评估时长: 30.0 秒
距离阈值: 2.0 米

==================================================
算法性能统计
==================================================

M-detector:
  Precision:  0.8234 ± 0.0456
  Recall:     0.8756 ± 0.0389
  F1-Score:   0.8487 ± 0.0412
  ALE:        0.2345 ± 0.0678 米
  
FAPP:
  Precision:  0.7845 ± 0.0523
  Recall:     0.8234 ± 0.0467
  F1-Score:   0.8034 ± 0.0489
  ALE:        0.3123 ± 0.0892 米

LV-DOT:
  Precision:  0.9123 ± 0.0312
  Recall:     0.8867 ± 0.0345
  F1-Score:   0.8993 ± 0.0328
  ALE:        0.1876 ± 0.0534 米

LDOT:
  Precision:  0.8756 ± 0.0389
  Recall:     0.9234 ± 0.0298
  F1-Score:   0.8989 ± 0.0334
  ALE:        0.2134 ± 0.0612 米

==================================================
算法排名
==================================================

按 F1-Score 排名:
  1. LV-DOT:     0.8993
  2. LDOT:       0.8989
  3. M-detector: 0.8487
  4. FAPP:       0.8034

按 ALE 排名（越小越好）:
  1. LV-DOT:     0.1876 米
  2. LDOT:       0.2134 米
  3. M-detector: 0.2345 米
  4. FAPP:       0.3123 米

==================================================
```

### 可视化图表

自动生成的图表文件（PNG 格式）：

1. **precision_comparison.png**：精确率对比曲线
2. **recall_comparison.png**：召回率对比曲线
3. **f1_comparison.png**：F1 分数对比曲线
4. **ale_comparison.png**：平均位置误差对比曲线

每个图表包含：
- 4 条曲线（4 个算法）
- X 轴：时间或帧数
- Y 轴：对应指标值
- 图例：算法名称和平均值

## 常见问题

### Q1: 如何理解 Precision 和 Recall 的权衡？

**场景分析：**

**高 Precision、低 Recall（保守型）：**
```
真值: 10 个物体
检测: 5 个物体，全部正确
结果: TP=5, FP=0, FN=5
      Precision=1.0, Recall=0.5
```
- 特点：只检测非常确定的物体，宁可漏检也不误报
- 适用：对误报敏感的场景（如自动驾驶）

**低 Precision、高 Recall（激进型）：**
```
真值: 10 个物体
检测: 15 个物体，包含所有真值
结果: TP=10, FP=5, FN=0
      Precision=0.67, Recall=1.0
```
- 特点：尽可能检测所有物体，可能产生误报
- 适用：对漏检敏感的场景（如安全监控）

**平衡型（理想）：**
```
真值: 10 个物体
检测: 11 个物体，包含 9 个真值
结果: TP=9, FP=2, FN=1
      Precision=0.82, Recall=0.90
```
- 特点：平衡误报和漏检
- 适用：大多数应用场景

### Q2: F1-Score 为什么重要？

**原因：**
1. **综合评估**：同时考虑 Precision 和 Recall
2. **避免偏颇**：防止只优化单一指标
3. **便于对比**：用一个数值比较算法优劣

**示例对比：**
```
算法 A: Precision=0.95, Recall=0.60, F1=0.74
算法 B: Precision=0.80, Recall=0.85, F1=0.82
```
虽然 A 的 Precision 更高，但 B 的综合性能（F1）更好。

### Q3: ALE 多大算好？

**参考标准：**
- < 0.2m：优秀，定位非常精确
- 0.2-0.5m：良好，满足大多数应用
- 0.5-1.0m：一般，可能需要优化
- > 1.0m：较差，定位精度不足

**影响因素：**
- 传感器精度（激光雷达分辨率）
- 聚类算法参数
- 物体大小和形状
- 遮挡情况

### Q4: 为什么有些帧的指标为 0？

**可能原因：**

1. **场景中无动态物体**
   - 真值数量 = 0
   - 所有指标无法计算

2. **算法未检测到任何物体**
   - 检测数量 = 0
   - Precision 和 Recall = 0

3. **完全漏检**
   - TP = 0
   - Recall = 0

**解决方法：**
- 检查 Gazebo 场景是否有动态物体
- 确认算法是否正常运行
- 查看算法输出话题是否有数据

### Q5: 如何提高算法的 Recall？

**方法：**

1. **降低检测阈值**
   - 更容易触发检测
   - 可能增加误报

2. **优化聚类参数**
   - 减小 `dbscan_eps`
   - 减小 `min_samples`

3. **扩大检测范围**
   - 增加传感器视野
   - 减少遮挡

4. **改进算法逻辑**
   - 使用更敏感的检测方法
   - 增加多帧融合

### Q6: 如何提高算法的 Precision？

**方法：**

1. **提高检测阈值**
   - 只保留高置信度检测
   - 可能增加漏检

2. **增加过滤条件**
   - 大小过滤
   - 速度过滤
   - 形状过滤

3. **优化聚类参数**
   - 增大 `min_samples`
   - 过滤小簇

4. **使用跟踪**
   - 多帧确认
   - 减少瞬时误检

### Q7: 评估结果不稳定怎么办？

**检查项：**

1. **评估时长是否足够**
   - 增加 `max_frames`
   - 至少评估 300 帧

2. **场景是否稳定**
   - 动态物体数量是否变化
   - 是否有遮挡情况

3. **算法是否稳定**
   - 查看算法日志
   - 检查参数配置

4. **距离阈值是否合适**
   - 过小：匹配不稳定
   - 过大：误匹配增加

### Q8: 如何对比不同参数配置的效果？

**方法：**

1. **固定评估条件**
   ```yaml
   max_frames: 600  # 固定帧数
   distance_threshold: 2.0  # 固定阈值
   ```

2. **使用相同场景**
   - 录制 rosbag
   - 重复播放相同数据

3. **多次测试取平均**
   - 每个配置测试 3-5 次
   - 计算平均值和标准差

4. **记录所有参数**
   - 创建测试记录表
   - 便于后续分析

## 高级功能

### 质心补偿

**问题**：激光雷达只能看到物体的一面，点云质心偏向传感器

**解决**：根据传感器位置估算物体真实中心

**配置：**
```yaml
enable_centroid_compensation: true
odom_topic: "/Odometry"
lidar_extrinsics: [-0.005, 0.0, 0.06, 0.0, 0.0, 0.0]
```

**效果：**
- 减小 ALE
- 提高定位精度
- 更准确的匹配

### 自适应聚类

**问题**：不同算法输出的点云密度不同

**解决**：为每个算法配置独立的聚类参数

**配置：**
```yaml
# M-detector 使用较大参数（点云稀疏）
mdetector_dbscan_eps: 0.5
mdetector_dbscan_min_samples: 10

# 其他算法使用较小参数（点云密集）
# 在代码中单独配置
```

### RViz 可视化

**功能**：实时显示检测结果和真值对比

**话题：**
- `/evaluation/ground_truth_markers`：真值标记（绿色）
- `/evaluation/detection_markers`：检测标记（红色）
- `/evaluation/matched_markers`：匹配标记（蓝色）

**使用：**
1. 打开 RViz
2. 添加 MarkerArray 显示
3. 订阅上述话题

## 技术细节

### 数据流程

```
1. 订阅话题
   ├─ Gazebo 真值: /gazebo/model_states
   ├─ M-detector: /m_detector/point_out
   ├─ FAPP: /dynamic_points
   ├─ LV-DOT: /onboard_detector/dynamic_point_cloud
   └─ LDOT: /ldot_detector/dynamic_point_cloud

2. 数据适配
   ├─ 提取动态物体位置（真值）
   ├─ 聚类点云（检测结果）
   └─ 转换为统一格式

3. 数据关联
   ├─ 计算距离矩阵
   ├─ 最近邻匹配
   └─ 分类 TP/FP/FN

4. 指标计算
   ├─ Precision = TP/(TP+FP)
   ├─ Recall = TP/(TP+FN)
   ├─ F1 = 2PR/(P+R)
   └─ ALE = Σdist/TP

5. 结果输出
   ├─ 保存 CSV 文件
   ├─ 生成可视化图表
   └─ 输出汇总报告
```

### 聚类算法

使用 DBSCAN（Density-Based Spatial Clustering）：

**参数：**
- `eps`：邻域半径
- `min_samples`：最小点数

**过程：**
1. 遍历所有点
2. 找到密度可达的点
3. 形成簇
4. 计算簇的质心

**优点：**
- 不需要预设簇数量
- 可以发现任意形状的簇
- 自动过滤噪声点

### 匹配算法

使用贪心最近邻匹配：

**伪代码：**
```python
for each detection:
    找到最近的未匹配真值点
    if 距离 < threshold:
        标记为 TP
        标记真值点为已匹配
    else:
        标记为 FP

for each 未匹配的真值点:
    标记为 FN
```

**复杂度**：O(n×m)，n=检测数，m=真值数

## 最佳实践

### 1. 评估前准备

**环境检查：**
- Gazebo 正常运行
- 动态物体正常移动
- 所有算法正常发布数据

**参数配置：**
- 根据场景调整距离阈值
- 设置合适的评估帧数
- 确认话题名称正确

### 2. 评估过程监控

**实时检查：**
- 观察终端输出的实时指标
- 检查 RViz 中的可视化
- 确认数据正常更新

**异常处理：**
- 指标异常 → 检查算法输出
- 无数据 → 检查话题连接
- 程序崩溃 → 查看错误日志

### 3. 结果分析

**数据验证：**
- 检查 CSV 文件完整性
- 确认评估帧数正确
- 验证指标范围合理

**对比分析：**
- 使用 F1-Score 综合排名
- 分析各算法的优劣势
- 结合 ALE 评估定位精度

### 4. 报告撰写

**包含内容：**
- 评估环境和参数
- 各算法性能数据
- 可视化对比图表
- 结论和建议

## 故障排除

### 问题 1：评估器无法启动

**检查：**
- Python 依赖是否安装（numpy, matplotlib）
- ROS 环境是否正确配置
- 配置文件是否存在

### 问题 2：无法接收真值数据

**检查：**
- Gazebo 是否运行
- 话题名称是否正确：`rostopic list | grep gazebo`
- 动态物体关键词是否匹配

### 问题 3：无法接收检测数据

**检查：**
- 算法节点是否运行：`rosnode list`
- 话题名称是否正确：`rostopic list`
- 话题是否有数据：`rostopic echo <topic_name> -n 1`

### 问题 4：所有指标都是 0

**原因：**
- 场景中无动态物体
- 算法未检测到物体
- 距离阈值设置过小

**解决：**
- 确认场景中有动态物体
- 检查算法输出
- 适当增大距离阈值

### 问题 5：ALE 异常大

**原因：**
- 聚类参数不合适
- 质心补偿未启用
- 传感器外参不正确

**解决：**
- 调整聚类参数
- 启用质心补偿
- 校准传感器外参

## 相关文件

- **启动文件**：`evaluation/launch/evaluation_gazebo.launch`
- **配置文件**：`evaluation/config/eval_config.yaml`
- **主脚本**：`evaluation/scripts/multi_algo_evaluator.py`
- **适配器**：`evaluation/scripts/detection_adapter.py`
- **关联器**：`evaluation/scripts/data_association.py`
- **绘图器**：`evaluation/scripts/plot_generator.py`
- **结果目录**：`evaluation/results/eval_*/`

## 扩展建议

### 1. 更多性能指标
- IoU（交并比）
- 跟踪连续性
- 检测延迟

### 2. 实时性能分析
- 处理时间统计
- 频率分析
- 延迟分布

### 3. 场景分类评估
- 按物体数量分类
- 按运动速度分类
- 按遮挡程度分类

### 4. 自动化测试
- 批量场景测试
- 参数自动优化
- 回归测试

### 5. 在线评估
- 实时指标显示
- 动态阈值调整
- 交互式可视化
