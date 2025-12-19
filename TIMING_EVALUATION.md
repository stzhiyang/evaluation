# 算法耗时评估功能

## 概述

算法耗时评估功能用于分析和对比多个算法的运行时间性能。该功能通过读取算法运行时记录的 CSV 耗时数据文件，自动计算平均耗时并生成详细的对比报告。

## 功能特点

### 1. 数据有效性验证
- **最小数据量要求**：每个算法至少需要 100 组有效数据才能进行计算
- **数据有效性判断**：除 `Timestamp` 和 `PointCount` 外，所有时间变量必须大于 0.001 秒
- **自动过滤**：自动过滤掉不符合条件的无效数据

### 2. 精确计算方法
- **计算范围**：取倒数 100 组有效数据进行计算
- **计算指标**：对 `TotalTime` 列求平均值
- **避免冷启动影响**：使用倒数数据可以排除算法启动初期的不稳定耗时

### 3. 多算法对比
- 支持同时评估多个算法（默认配置 4 个）
- 自动生成算法间的耗时对比
- 清晰展示每个算法的性能表现

### 4. 详细报告生成
- 自动生成带时间戳的 TXT 报告文件
- 包含每个算法的详细统计信息
- 提供成功/失败状态和错误信息

## 文件结构

```
evaluation/
├── launch/
│   └── timing_calculation.launch    # 启动配置文件
├── scripts/
│   └── calculate_timing.py          # 计算脚本
└── results/
    └── timing_reports/              # 报告输出目录
        └── timing_report_YYYYMMDD_HHMMSS.txt
```

## 使用方法

### 步骤 1：配置 CSV 文件路径

编辑 `evaluation/launch/timing_calculation.launch` 文件，修改四个算法的 CSV 文件路径：

```xml
<rosparam param="csv_files">
    LDOT: "$(find evaluation)/../LDOT/ldot_detector/timing/test_ldot_timing_20251219_174821_680.csv"
    M-detector: "$(find evaluation)/../M-detector/timing/test_mdetector_timing_20251219_210219_424.csv"
    LV-DOT: "$(find evaluation)/../LV-DOT/onboard_detector/timing/test_lvdot_timing_YYYYMMDD_HHMMSS.csv"
    FAPP: "$(find evaluation)/../FAPP/timing/test_fapp_timing_YYYYMMDD_HHMMSS.csv"
</rosparam>
```

**配置说明：**
- 键名（如 `LDOT`）：算法名称，会显示在报告中
- 值：CSV 文件的完整路径
- 支持使用 `$(find package_name)` 来引用 ROS 包路径
- 可以添加或删除算法，不限于 4 个

### 步骤 2：配置输出目录（可选）

如需修改报告输出位置，编辑 launch 文件中的 `output_dir` 参数：

```xml
<param name="output_dir" value="$(find evaluation)/results/timing_reports" />
```

### 步骤 3：运行计算

在终端执行：

```bash
roslaunch evaluation timing_calculation.launch
```

### 步骤 4：查看结果

计算完成后，报告会自动保存到指定目录，文件名格式为：
```
timing_report_YYYYMMDD_HHMMSS.txt
```

例如：`timing_report_20251219_153045.txt`

## CSV 文件格式要求

### 必需列
- `TotalTime`：总耗时（秒），这是计算的目标列

### 可选列
- `Timestamp`：时间戳（会被排除在有效性检查之外）
- `PointCount`：点云数量（会被排除在有效性检查之外）
- 其他时间列：如 `DetectionTime`、`ClusteringTime` 等

### 示例格式

**LDOT 格式：**
```csv
Timestamp,PreprocessTime,DetectionTime,TrackingTime,ClassificationTime,TotalTime
3801.965000000,0.177853,1.04793,0.035616,0.024708,1.36446
3802.066000000,0.333683,1.27343,0.135246,0.027485,1.82955
...
```

**M-detector 格式：**
```csv
Timestamp,PointCount,DetectionTime,ClusteringTime,BufferTime,DepthMapTime,TotalTime
10189.194000000,3720,1.637023001,0.570318996,0.051220006,0.003556008,3.237450001
10189.292051056,3721,0.548623007,0.574203004,0.066916007,0.004116999,1.657582994
...
```

## 报告内容说明

### 报告头部
```
==================================================
算法耗时计算报告
==================================================

计算时间: 2025-12-19 15:30:45
计算条件: 至少 100 组有效数据（所有时间变量 > 0.001 秒）
计算方法: 取倒数 100 组有效数据的 TotalTime 平均值
```

### 单个算法结果
```
算法: LDOT
文件: LDOT/ldot_detector/timing/test_ldot_timing_20251219_174821_680.csv
状态: ✓ 计算成功
平均耗时: 4.123456 秒
有效数据: 185 组
总数据量: 200 组
```

### 汇总统计
```
==================================================
汇总统计
==================================================

总算法数: 4
成功计算: 3
计算失败: 1

成功算法耗时对比:
  LDOT                : 4.123456 秒
  M-detector          : 35.678901 秒
  LV-DOT              : 2.345678 秒
```

## 数据有效性规则

### 有效数据定义
一组数据被认为是有效的，需要满足以下条件：

1. **所有时间列的值 > 0.001 秒**
   - 包括：`PreprocessTime`、`DetectionTime`、`TrackingTime`、`TotalTime` 等
   - 排除：`Timestamp`、`PointCount`（这些不是时间测量值）

2. **数据格式正确**
   - 所有数值列可以成功转换为浮点数
   - 没有缺失值或非法字符

### 无效数据示例

```csv
# 示例 1：某个时间列为 0（无效）
Timestamp,DetectionTime,ClusteringTime,TotalTime
100.0,0.0,0.5,0.5

# 示例 2：某个时间列小于 0.001（无效）
Timestamp,DetectionTime,ClusteringTime,TotalTime
101.0,0.0005,0.5,0.5005

# 示例 3：所有时间列都大于 0.001（有效）
Timestamp,DetectionTime,ClusteringTime,TotalTime
102.0,0.002,0.5,0.502
```

## 常见问题

### Q1: 为什么我的算法计算失败？

**可能原因：**
1. CSV 文件路径不正确
2. 文件中缺少 `TotalTime` 列
3. 有效数据不足 100 组
4. 文件格式错误或损坏

**解决方法：**
- 检查 launch 文件中的路径是否正确
- 确认 CSV 文件包含 `TotalTime` 列
- 查看报告中的错误信息获取详细原因

### Q2: 为什么要求至少 100 组有效数据？

**原因：**
- 确保统计结果的可靠性
- 排除偶然因素的影响
- 提供足够的样本量进行平均计算

### Q3: 为什么使用倒数 100 组数据而不是全部数据？

**原因：**
- 算法启动初期可能存在冷启动效应
- 系统资源分配可能需要时间稳定
- 倒数数据更能反映算法的稳定运行性能

### Q4: 可以同时评估超过 4 个算法吗？

**可以！** 在 launch 文件中添加更多算法配置即可：

```xml
<rosparam param="csv_files">
    Algorithm1: "/path/to/csv1.csv"
    Algorithm2: "/path/to/csv2.csv"
    Algorithm3: "/path/to/csv3.csv"
    Algorithm4: "/path/to/csv4.csv"
    Algorithm5: "/path/to/csv5.csv"
    # 可以继续添加...
</rosparam>
```

### Q5: 如何修改有效数据的阈值（0.001 秒）？

需要修改 `calculate_timing.py` 脚本中的 `is_valid_row()` 函数：

```python
def is_valid_row(row, exclude_columns=['Timestamp', 'PointCount']):
    # 修改这里的阈值
    threshold = 0.001  # 改为你需要的值
    
    for key, value in row.items():
        if key in exclude_columns:
            continue
        if float(value) <= threshold:  # 使用新阈值
            return False
    return True
```

### Q6: 如何修改计算的数据组数（100 组）？

需要修改 `calculate_timing.py` 脚本中的两处：

```python
# 1. 最小数据量检查
if len(valid_rows) < 100:  # 改为你需要的最小值
    result['error'] = f"有效数据不足 100 组..."
    return result

# 2. 计算范围
last_100 = valid_rows[-100:]  # 改为你需要的组数
result['average_time'] = sum(last_100) / len(last_100)
```

## 技术细节

### 脚本工作流程

1. **初始化 ROS 节点**
   - 节点名称：`timing_calculator`
   - 从参数服务器读取配置

2. **读取 CSV 文件**
   - 使用 Python `csv.DictReader` 解析文件
   - 逐行检查数据有效性

3. **数据验证**
   - 检查必需列是否存在
   - 验证数值格式
   - 统计有效数据数量

4. **计算平均值**
   - 提取倒数 100 组有效数据
   - 对 `TotalTime` 求和并除以 100

5. **生成报告**
   - 创建输出目录
   - 生成带时间戳的文件名
   - 写入格式化的报告内容

### 依赖项

- **Python 3**
- **ROS (rospy)**
- **Python 标准库**：
  - `csv`：CSV 文件解析
  - `os`：文件系统操作
  - `datetime`：时间戳生成
  - `pathlib`：路径处理

### 性能考虑

- **内存使用**：所有有效数据会加载到内存中
- **处理速度**：取决于 CSV 文件大小，通常在秒级完成
- **并发性**：当前为单线程处理，按顺序处理每个算法

## 扩展建议

### 可能的改进方向

1. **支持更多统计指标**
   - 中位数、标准差、最大值、最小值
   - 百分位数（P50、P95、P99）

2. **可视化功能**
   - 生成耗时对比图表
   - 时间序列趋势图

3. **自动化测试**
   - 批量处理多个测试场景
   - 自动对比不同版本的性能

4. **实时监控**
   - 订阅 ROS 话题实时计算
   - 动态更新统计结果

5. **配置文件支持**
   - 使用 YAML 配置文件代替 launch 参数
   - 支持更复杂的配置选项

## 相关文件

- **启动文件**：`evaluation/launch/timing_calculation.launch`
- **计算脚本**：`evaluation/scripts/calculate_timing.py`
- **报告目录**：`evaluation/results/timing_reports/`
- **示例 CSV**：
  - `LDOT/ldot_detector/timing/*.csv`
  - `M-detector/timing/*.csv`
  - `LV-DOT/onboard_detector/timing/*.csv`
  - `FAPP/timing/*.csv`

## 联系与支持

如有问题或建议，请查看：
- 报告文件中的详细错误信息
- ROS 日志输出（`rosout`）
- 脚本源码中的注释说明
