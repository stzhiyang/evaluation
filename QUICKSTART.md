# 快速开始指南

## 前置条件检查

### 1. 测试系统是否正常
```bash
cd /home/st/M_detector/src/M-detector/evaluation/scripts
python3 test_system.py
```

如果看到 "✅ All tests passed!" 说明系统已准备就绪。

### 2. 编译工作空间
```bash
cd /home/st/M_detector
catkin_make
source devel/setup.bash
```

## 使用方法

### 方法一: 手动启动(推荐用于调试)

#### 步骤1: 启动Gazebo仿真
```bash
# 终端1
source /home/st/FAPP/devel/setup.bash
roslaunch mot_mapping sim_mapping.launch obj_num:=5
```

#### 步骤2: 启动M-detector
```bash
# 终端2
source /home/st/M_detector/devel/setup.bash
roslaunch m_detector detector_avia.launch
```

#### 步骤3: 启动FAPP
```bash
# 终端3
source /home/st/FAPP/devel/setup.bash
# FAPP通常随仿真自动启动,如需独立启动:
# roslaunch mot_mapping mapping_sim.launch
```

#### 步骤4: 启动LV-DOT
```bash
# 终端4
source /home/st/ODOT/devel/setup.bash
roslaunch onboard_detector run_detector_sim.launch
```

#### 步骤5: 启动评估系统
```bash
# 终端5
source /home/st/M_detector/devel/setup.bash
# 不启动RViz(推荐,避免与LV-DOT/M-detector的RViz冲突)
roslaunch evaluation evaluation_gazebo.launch

# 或者如需启动评估专用RViz:
# roslaunch evaluation evaluation_gazebo.launch rviz:=true
```

#### 步骤6: (可选) 查看RViz可视化
评估系统默认不启动RViz,避免与其他系统冲突。

**如需可视化,有两种方式:**

**方式A: 使用现有RViz** (推荐)
在LV-DOT或M-detector的RViz中添加评估话题:
- Add → MarkerArray → Topic: `/evaluation/ground_truth`
- Add → MarkerArray → Topic: `/evaluation/mdetector_detections`
- Add → MarkerArray → Topic: `/evaluation/fapp_detections`
- Add → MarkerArray → Topic: `/evaluation/lvdot_detections`

**方式B: 单独启动评估RViz**
```bash
# 新终端
rviz -d /home/st/M_detector/src/M-detector/evaluation/rviz/evaluation.rviz
```

**可视化说明:**
- **绿色立方体**: Gazebo真值
- **红色立方体**: M-detector检测结果
- **蓝色立方体**: FAPP检测结果
- **绿色立方体**: LV-DOT检测结果(注意与真值颜色区分透明度)
- **橙色立方体**: 第四算法检测结果
- **白色文字**: 实时指标显示

#### 步骤7: 停止并生成报告
在评估节点终端按 `Ctrl+C`,系统会自动:
1. 保存所有数据
2. 生成对比图表
3. 打印性能总结

结果保存在: `/home/st/M_detector/src/M-detector/evaluation/results/`

---

### 方法二: 使用tmux脚本(一键启动)

```bash
cd /home/st/M_detector/src/M-detector/evaluation
./start_evaluation.sh
```

这会创建一个tmux会话,包含所有组件的窗口。

**tmux快捷键:**
- `Ctrl+B` 然后按 `0-5`: 切换到对应窗口
- `Ctrl+B` 然后按 `D`: 脱离会话(后台运行)
- `tmux attach -t eval_system`: 重新连接会话
- `Ctrl+C` (在各窗口): 停止对应进程

---

## 配置调整

### 修改评估参数
编辑 `config/eval_config.yaml`:

```yaml
# 匹配阈值
distance_threshold: 2.0      # 降低会更严格
iou_threshold: 0.1           # 提高会更严格

# 更新频率
update_freq: 10.0            # 提高会更频繁评估

# 话题名称(根据实际调整)
mdetector_topic: "/dynamic_points"
fapp_topic: "/states"
lvdot_topic: "/onboard_detector/dynamic_bboxes"

# Gazebo过滤关键词(根据你的模型命名)
dynamic_keywords: ["actor", "dynamic", "person", "obstacle", "sphere", "box"]
```

### 修改M-detector聚类参数
如果M-detector检测效果不好,调整:

```yaml
mdetector_dbscan_eps: 0.5           # 聚类半径
mdetector_dbscan_min_samples: 10   # 最小点数
```

---

## 查看结果

### 1. 实时监控

在RViz中查看实时可视化,或订阅话题:
```bash
# 查看实时指标
rostopic echo /evaluation/metrics_text

# 查看检测结果
rostopic echo /evaluation/ground_truth
rostopic echo /evaluation/mdetector_detections
rostopic echo /evaluation/fapp_detections
rostopic echo /evaluation/lvdot_detections
```

### 2. 结果文件

停止评估后,查看 `results/` 目录:

```bash
cd /home/st/M_detector/src/M-detector/evaluation/results

# 查看总体报告
cat summary.json

# 查看性能表格
cat performance_table.txt

# 查看逐帧数据
head frame_metrics.csv

# 查看图表
eog time_series.png
eog radar_chart.png
eog performance_table.png
```

---

## 常见问题排查

### Q1: 某个算法没有检测输出

**检查话题:**
```bash
rostopic list | grep -E "dynamic|states|bboxes"
```

**解决:** 在 `config/eval_config.yaml` 中修改对应话题名称

### Q2: 评估节点报错 "No module named 'obj_state_msgs'"

**解决:** 编译FAPP工作空间
```bash
cd /home/st/FAPP
catkin_make
source devel/setup.bash
```

### Q3: Gazebo没有动态物体

**检查模型:**
```bash
rostopic echo /gazebo/model_states
```

**解决:** 确保启动仿真时包含动态物体,或调整 `dynamic_keywords`

### Q4: RViz中看不到可视化

**检查发布频率:**
```bash
rostopic hz /evaluation/ground_truth
```

**解决:** 
1. 确保有Gazebo真值数据
2. 检查 `enable_visualization: true` 在配置中
3. 检查RViz的Fixed Frame设置为 "world"

### Q5: 生成的图表是空的

**原因:** 评估时间太短,数据不足

**解决:** 运行至少30秒后再停止评估

---

## 离线评估rosbag

### 录制数据
```bash
rosbag record \
  /gazebo/model_states \
  /dynamic_points \
  /states \
  /onboard_detector/dynamic_bboxes \
  -O evaluation_test.bag
```

### 重放评估
```bash
# 终端1: 启动评估节点
roslaunch evaluation evaluation_gazebo.launch rviz:=false

# 终端2: 重放bag
rosbag play evaluation_test.bag --clock

# Ctrl+C停止评估并生成报告
```

---

## 批量评估多个场景

创建脚本 `batch_eval.sh`:

```bash
#!/bin/bash
SCENARIOS=("3obj" "5obj" "10obj")
for scenario in "${SCENARIOS[@]}"; do
    echo "Evaluating $scenario..."
    
    # 修改Gazebo参数
    rosparam set /obj_num ${scenario:0:1}
    
    # 运行评估30秒
    timeout 30 roslaunch evaluation evaluation_gazebo.launch
    
    # 重命名结果
    mv results results_$scenario
done
```

---

## 性能优化建议

1. **降低更新频率**: 如果计算资源有限,设置 `update_freq: 5.0`
2. **关闭可视化**: 设置 `enable_visualization: false`
3. **使用rosbag**: 先录制再离线评估,避免实时压力
4. **调整匹配阈值**: 根据场景大小调整 `distance_threshold`

---

## 下一步

- 阅读 `README.md` 了解更多技术细节
- 查看 `scripts/` 目录了解代码实现
- 根据你的第四算法修改 `config/eval_config.yaml` 中的 `algo4_topic`
- 根据实际场景调整 `dynamic_keywords` 和 `default_bbox_size`

祝评估顺利! 🚀
