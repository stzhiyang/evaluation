#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-Algorithm Evaluator Node
多算法动态物体检测跟踪评估主节点
"""

import rospy
import numpy as np
import json
import csv
from collections import deque, defaultdict
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import sys
import os
from datetime import datetime

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from detection_adapter import DetectionAdapter, StandardDetection
from data_association import DataAssociator, MetricsCalculator
from plot_generator import PlotGenerator

# 所有算法统一使用PointCloud2格式,不再需要特殊消息类型


class MultiAlgorithmEvaluator:
    """多算法评估器"""
    
    def __init__(self):
        rospy.init_node('multi_algorithm_evaluator', anonymous=False)
        
        # 参数配置
        self.load_parameters()
        
        # 初始化模块
        self.adapter = DetectionAdapter()
        self.associator = DataAssociator(
            distance_threshold=self.distance_threshold
        )
        
        # 为每个算法创建计算器
        self.calculators = {
            'M-detector': MetricsCalculator(),
            'FAPP': MetricsCalculator(),
            'LV-DOT': MetricsCalculator(),
            'LDOT': MetricsCalculator()
        }
        
        # 数据缓冲区
        self.gt_buffer = deque(maxlen=100)
        self.detection_buffers = {
            'M-detector': deque(maxlen=100),
            'FAPP': deque(maxlen=100),
            'LV-DOT': deque(maxlen=100),
            'LDOT': deque(maxlen=100)
        }
        
        # 时间戳和指标记录
        self.algo_metrics_dict = {
            'M-detector': {'timestamps': [], 'metrics': []},
            'FAPP': {'timestamps': [], 'metrics': []},
            'LV-DOT': {'timestamps': [], 'metrics': []},
            'LDOT': {'timestamps': [], 'metrics': []}
        }
        
        # 当前真值
        self.current_gt = []
        
        # 订阅器
        self.setup_subscribers()
        
        # 发布器
        self.setup_publishers()
        
        # 定时器
        self.eval_timer = rospy.Timer(rospy.Duration(1.0 / self.update_freq), self.evaluate_callback)
        
        # 启动时间
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("Multi-Algorithm Evaluator initialized")
        rospy.loginfo(f"Algorithms: M-detector, FAPP, LV-DOT, LDOT")
        rospy.loginfo(f"Update frequency: {self.update_freq} Hz")
        rospy.loginfo(f"Results will be saved to: {self.output_dir}")
        
        # 打印评估控制信息
        if self.max_frames > 0:
            rospy.loginfo(f"Max evaluation frames: {self.max_frames}")
    
    def load_parameters(self):
        """加载参数"""
        # 匹配阈值（只使用距离）
        self.distance_threshold = rospy.get_param('~distance_threshold', 2.0)
        
        # 更新频率
        self.update_freq = rospy.get_param('~update_freq', 10.0)
        
        # 评估控制参数 - 新增
        self.max_frames = rospy.get_param('~max_frames', 0)  # 最大评估帧数，0表示无限制
        self.frame_count = 0  # 当前已评估帧数
        
        # 输出目录 - 修改为自动创建带时间戳的子文件夹
        base_output_dir = rospy.get_param('~output_dir', 
                                         os.path.join(os.path.dirname(__file__), '../results'))
        
        # 创建带时间戳的子文件夹和文件名前缀
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.output_dir = os.path.join(base_output_dir, f'eval_{self.timestamp}')
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # 话题名称
        self.gt_topic = rospy.get_param('~ground_truth_topic', '/gazebo/model_states')
        self.mdetector_topic = rospy.get_param('~mdetector_topic', '/dynamic_points')
        self.fapp_topic = rospy.get_param('~fapp_topic', '/states')
        self.lvdot_topic = rospy.get_param('~lvdot_topic', '/onboard_detector/dynamic_bboxes')
        self.ldot_topic = rospy.get_param('~ldot_topic', '/ldot_detector/dynamic_point_cloud')
        
        # Gazebo动态物体过滤关键词
        self.dynamic_keywords = rospy.get_param('~dynamic_keywords', ['actor', 'dynamic', 'person', 'obstacle'])
        
        # 物体尺寸估计(根据Gazebo模型类型)
        self.default_bbox_size = rospy.get_param('~default_bbox_size', [0.5, 0.5, 1.5])
    
    def setup_subscribers(self):
        """设置订阅器"""
        # 真值
        self.gt_sub = rospy.Subscriber(self.gt_topic, ModelStates, self.gt_callback, queue_size=10)
        
        # 所有算法统一订阅动态点云(PointCloud2),由评估器统一聚类生成边界框
        # M-detector
        self.mdetector_sub = rospy.Subscriber(self.mdetector_topic, PointCloud2, 
                                             self.mdetector_callback, queue_size=10)
        
        # FAPP
        self.fapp_sub = rospy.Subscriber(self.fapp_topic, PointCloud2, 
                                        self.fapp_callback, queue_size=10)
        
        # LV-DOT
        self.lvdot_sub = rospy.Subscriber(self.lvdot_topic, PointCloud2, 
                                         self.lvdot_callback, queue_size=10)
        
        # LDOT
        self.ldot_sub = rospy.Subscriber(self.ldot_topic, PointCloud2, 
                                         self.ldot_callback, queue_size=10)
    
    def setup_publishers(self):
        """设置发布器 - 始终发布可视化话题，由 launch 文件控制是否启动 RViz"""
        self.gt_marker_pub = rospy.Publisher('/evaluation/ground_truth', MarkerArray, queue_size=10)
        self.det_marker_pubs = {
            'M-detector': rospy.Publisher('/evaluation/mdetector_detections', MarkerArray, queue_size=10),
            'FAPP': rospy.Publisher('/evaluation/fapp_detections', MarkerArray, queue_size=10),
            'LV-DOT': rospy.Publisher('/evaluation/lvdot_detections', MarkerArray, queue_size=10),
            'LDOT': rospy.Publisher('/evaluation/ldot_detections', MarkerArray, queue_size=10)
        }
        self.match_line_pub = rospy.Publisher('/evaluation/match_lines', Marker, queue_size=10)
        self.metrics_text_pub = rospy.Publisher('/evaluation/metrics_text', MarkerArray, queue_size=10)
    
    def should_stop_evaluation(self):
        """检查是否应该停止评估"""
        # 检查帧数限制
        if self.max_frames > 0 and self.frame_count >= self.max_frames:
            rospy.loginfo(f"Reached max frames: {self.frame_count}/{self.max_frames}")
            return True
        
        return False
    
    def gt_callback(self, msg):
        """Gazebo真值回调"""
        detections = []
        
        for i, name in enumerate(msg.name):
            # 过滤动态物体
            is_dynamic = any(keyword in name.lower() for keyword in self.dynamic_keywords)
            
            if is_dynamic:
                # 估计边界框尺寸
                bbox_size = np.array(self.default_bbox_size)
                
                # Gazebo 的 position.z 是底部位置，需要转换为中心位置
                # 用于数据关联时计算准确的 3D 距离
                position = np.array([
                    msg.pose[i].position.x,
                    msg.pose[i].position.y,
                    msg.pose[i].position.z + bbox_size[2] / 2.0  # 转换为中心位置
                ])
                
                velocity = np.array([
                    msg.twist[i].linear.x,
                    msg.twist[i].linear.y,
                    msg.twist[i].linear.z
                ])
                
                detection = StandardDetection(
                    obj_id=i,
                    position=position,
                    velocity=velocity,
                    bbox_size=bbox_size,
                    timestamp=rospy.Time.now().to_sec()
                )
                detections.append(detection)
        
        self.current_gt = detections
        self.gt_buffer.append(detections)
    
    def mdetector_callback(self, msg):
        """M-detector回调 - 统一使用点云解析"""
        detections = self.adapter.parse_mdetector(msg)
        self.detection_buffers['M-detector'].append(detections)
    
    def fapp_callback(self, msg):
        """FAPP回调 - 统一使用点云解析"""
        detections = self.adapter.parse_mdetector(msg)  # 复用点云解析逻辑
        self.detection_buffers['FAPP'].append(detections)
    
    def lvdot_callback(self, msg):
        """LV-DOT回调 - 统一使用点云解析"""
        detections = self.adapter.parse_mdetector(msg)  # 复用点云解析逻辑
        self.detection_buffers['LV-DOT'].append(detections)
    
    def ldot_callback(self, msg):
        """LDOT回调 - 统一使用点云解析"""
        detections = self.adapter.parse_mdetector(msg)  # 复用点云解析逻辑
        self.detection_buffers['LDOT'].append(detections)
    
    def evaluate_callback(self, event):
        """评估定时回调"""
        if len(self.current_gt) == 0:
            return
        
        # 检查是否达到评估限制
        if self.should_stop_evaluation():
            rospy.loginfo("Evaluation limit reached. Stopping...")
            self.save_results()
            rospy.signal_shutdown("Evaluation completed")
            return
        
        current_time = rospy.Time.now().to_sec()
        
        # 评估每个算法
        for algo_name in ['M-detector', 'FAPP', 'LV-DOT', 'LDOT']:
            if len(self.detection_buffers[algo_name]) == 0:
                continue
            
            # 获取最新检测结果
            detections = self.detection_buffers[algo_name][-1]
            
            # 数据关联
            matches, unmatched_gt, unmatched_det = self.associator.associate(
                self.current_gt, detections
            )
            
            # 计算指标
            metrics = self.calculators[algo_name].compute_frame_metrics(
                matches, unmatched_gt, unmatched_det
            )
            
            # 记录
            self.algo_metrics_dict[algo_name]['timestamps'].append(current_time)
            self.algo_metrics_dict[algo_name]['metrics'].append(metrics)
            
            # 可视化
            self.visualize_detections(algo_name, detections, matches)
        
        # 增加帧计数
        self.frame_count += 1
        
        # 可视化真值和指标
        self.visualize_ground_truth()
        self.visualize_metrics()
    
    def visualize_ground_truth(self):
        """可视化真值"""
        marker_array = MarkerArray()
        
        for detection in self.current_gt:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "ground_truth"
            marker.id = detection.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # RViz 的 CUBE marker 位置是中心点
            # detection.position 已经是中心位置，直接使用
            marker.pose.position.x = detection.position[0]
            marker.pose.position.y = detection.position[1]
            marker.pose.position.z = detection.position[2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = detection.bbox_size[0]
            marker.scale.y = detection.bbox_size[1]
            marker.scale.z = detection.bbox_size[2]
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            marker.lifetime = rospy.Duration(0.5)
            
            marker_array.markers.append(marker)
        
        self.gt_marker_pub.publish(marker_array)
    
    def visualize_detections(self, algo_name, detections, matches):
        """可视化检测结果"""
        marker_array = MarkerArray()
        
        # 颜色映射 - 高对比度颜色，更容易区分
        colors = {
            'M-detector': (1.0, 0.0, 0.0, 0.7),  # 纯红色
            'FAPP': (0.0, 0.5, 1.0, 0.7),        # 亮蓝色
            'LV-DOT': (1.0, 1.0, 0.0, 0.7),      # 黄色
            'LDOT': (0.5, 0.0, 1.0, 0.7)         # 深紫色
        }
        
        color = colors.get(algo_name, (0.5, 0.5, 0.5, 0.6))
        
        for detection in detections:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = algo_name
            marker.id = detection.id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = detection.position[0]
            marker.pose.position.y = detection.position[1]
            marker.pose.position.z = detection.position[2]
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = detection.bbox_size[0]
            marker.scale.y = detection.bbox_size[1]
            marker.scale.z = detection.bbox_size[2]
            
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            
            marker.lifetime = rospy.Duration(0.5)
            
            marker_array.markers.append(marker)
        
        if algo_name in self.det_marker_pubs:
            self.det_marker_pubs[algo_name].publish(marker_array)
    
    def visualize_metrics(self):
        """可视化实时指标 - 横向并排显示"""
        marker_array = MarkerArray()
        
        # 算法颜色映射（与检测框颜色一致）
        algo_colors = {
            'M-detector': (1.0, 0.0, 0.0),  # 红色
            'FAPP': (0.0, 0.5, 1.0),        # 蓝色
            'LV-DOT': (1.0, 1.0, 0.0),      # 黄色
            'LDOT': (0.5, 0.0, 1.0)         # 深紫色
        }
        
        # 横向排列，x方向偏移
        x_offset = -6.0  # 起始x位置
        x_spacing = 4.0  # 每个算法之间的间距
        
        for i, algo_name in enumerate(['M-detector', 'FAPP', 'LV-DOT', 'LDOT']):
            if len(self.algo_metrics_dict[algo_name]['metrics']) == 0:
                continue
            
            latest_metrics = self.algo_metrics_dict[algo_name]['metrics'][-1]
            color = algo_colors.get(algo_name, (1.0, 1.0, 1.0))
            
            # 算法名称和指标文本
            text = f"{algo_name}\n"
            text += f"R:{latest_metrics['recall']:.2f} "
            text += f"P:{latest_metrics['precision']:.2f}\n"
            text += f"F1:{latest_metrics['f1']:.2f} "
            text += f"MOTP:{latest_metrics['motp']:.2f}m"
            
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "metrics_text"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # 横向排列位置
            marker.pose.position.x = x_offset + i * x_spacing
            marker.pose.position.y = 0.0
            marker.pose.position.z = 4.0  # 固定高度
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.4  # 稍大的字体
            
            # 使用算法对应的颜色
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            marker.text = text
            marker.lifetime = rospy.Duration(0.5)
            
            marker_array.markers.append(marker)
        
        self.metrics_text_pub.publish(marker_array)
    
    def save_results(self):
        """保存评估结果"""
        rospy.loginfo("Saving evaluation results...")
        
        # 保存逐帧指标到CSV
        self.save_frame_metrics_csv()
        
        # 计算并保存总体指标
        overall_metrics = {}
        for algo_name, calculator in self.calculators.items():
            overall_metrics[algo_name] = calculator.compute_overall_metrics()
        
        self.save_overall_metrics_json(overall_metrics)
        
        # 生成图表
        plot_gen = PlotGenerator(self.output_dir)
        
        try:
            plot_gen.plot_time_series(self.algo_metrics_dict, timestamp=self.timestamp)
        except Exception as e:
            rospy.logerr(f"Failed to generate time series plot: {e}")
        
        try:
            plot_gen.plot_radar_chart(overall_metrics, timestamp=self.timestamp)
        except Exception as e:
            rospy.logerr(f"Failed to generate radar chart: {e}")
        
        try:
            plot_gen.generate_performance_table(overall_metrics, timestamp=self.timestamp)
        except Exception as e:
            rospy.logerr(f"Failed to generate performance table: {e}")
        
        rospy.loginfo(f"Results saved to: {self.output_dir}")
        
        # 打印总结
        self.print_summary(overall_metrics)
    
    def save_frame_metrics_csv(self):
        """保存逐帧指标到CSV"""
        csv_path = os.path.join(self.output_dir, f'frame_metrics_{self.timestamp}.csv')
        
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Algorithm', 'TP', 'FP', 'FN', 'Recall', 'Precision', 'F1', 'MOTP'])
            
            for algo_name in ['M-detector', 'FAPP', 'LV-DOT', 'LDOT']:
                timestamps = self.algo_metrics_dict[algo_name]['timestamps']
                metrics_list = self.algo_metrics_dict[algo_name]['metrics']
                
                for ts, metrics in zip(timestamps, metrics_list):
                    writer.writerow([
                        ts,
                        algo_name,
                        metrics['tp'],
                        metrics['fp'],
                        metrics['fn'],
                        metrics['recall'],
                        metrics['precision'],
                        metrics['f1'],
                        metrics['motp']
                    ])
    
    def save_overall_metrics_json(self, overall_metrics):
        """保存总体指标到JSON"""
        json_path = os.path.join(self.output_dir, f'summary_{self.timestamp}.json')
        
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        summary = {
            'evaluation_time': rospy.Time.now().to_sec(),
            'start_time': self.start_time.to_sec(),
            'duration': elapsed_time,
            'total_frames': self.frame_count,
            'parameters': {
                'distance_threshold': self.distance_threshold,
                'update_freq': self.update_freq,
                'max_frames': self.max_frames
            },
            'algorithms': overall_metrics
        }
        
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2)
    
    def print_summary(self, overall_metrics):
        """打印评估总结"""
        elapsed_time = (rospy.Time.now() - self.start_time).to_sec()
        
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("EVALUATION SUMMARY")
        rospy.loginfo("="*80)
        rospy.loginfo(f"\nEvaluation Statistics:")
        rospy.loginfo(f"  Total Frames:  {self.frame_count}")
        rospy.loginfo(f"  Duration:      {elapsed_time:.2f} seconds")
        rospy.loginfo(f"  Output Dir:    {self.output_dir}")
        
        for algo_name, metrics in overall_metrics.items():
            rospy.loginfo(f"\n{algo_name}:")
            rospy.loginfo(f"  Recall:    {metrics['recall_mean']:.3f} ± {metrics['recall_std']:.3f}")
            rospy.loginfo(f"  Precision: {metrics['precision_mean']:.3f} ± {metrics['precision_std']:.3f}")
            rospy.loginfo(f"  F1-Score:  {metrics['f1_mean']:.3f} ± {metrics['f1_std']:.3f}")
            rospy.loginfo(f"  MOTP:      {metrics['motp_mean']:.3f} ± {metrics['motp_std']:.3f} m")
            rospy.loginfo(f"  Total TP:  {metrics['total_tp']}")
            rospy.loginfo(f"  Total FP:  {metrics['total_fp']}")
            rospy.loginfo(f"  Total FN:  {metrics['total_fn']}")
        
        rospy.loginfo("\n" + "="*80)
    
    def run(self):
        """运行评估器"""
        rospy.loginfo("Multi-Algorithm Evaluator is running...")
        rospy.loginfo("Press Ctrl+C to stop and save results")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down...")
        finally:
            self.save_results()


if __name__ == '__main__':
    try:
        evaluator = MultiAlgorithmEvaluator()
        evaluator.run()
    except rospy.ROSInterruptException:
        pass
