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
            distance_threshold=self.distance_threshold,
            iou_threshold=self.iou_threshold
        )
        
        # 为每个算法创建计算器
        self.calculators = {
            'M-detector': MetricsCalculator(),
            'FAPP': MetricsCalculator(),
            'LV-DOT': MetricsCalculator(),
            'Algorithm4': MetricsCalculator()
        }
        
        # 数据缓冲区
        self.gt_buffer = deque(maxlen=100)
        self.detection_buffers = {
            'M-detector': deque(maxlen=100),
            'FAPP': deque(maxlen=100),
            'LV-DOT': deque(maxlen=100),
            'Algorithm4': deque(maxlen=100)
        }
        
        # 时间戳和指标记录
        self.algo_metrics_dict = {
            'M-detector': {'timestamps': [], 'metrics': []},
            'FAPP': {'timestamps': [], 'metrics': []},
            'LV-DOT': {'timestamps': [], 'metrics': []},
            'Algorithm4': {'timestamps': [], 'metrics': []}
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
        rospy.loginfo(f"Algorithms: M-detector, FAPP, LV-DOT, Algorithm4")
        rospy.loginfo(f"Update frequency: {self.update_freq} Hz")
        rospy.loginfo(f"Results will be saved to: {self.output_dir}")
    
    def load_parameters(self):
        """加载参数"""
        # 匹配阈值
        self.distance_threshold = rospy.get_param('~distance_threshold', 2.0)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.1)
        
        # 更新频率
        self.update_freq = rospy.get_param('~update_freq', 10.0)
        
        # 输出目录
        self.output_dir = rospy.get_param('~output_dir', 
                                         os.path.join(os.path.dirname(__file__), '../results'))
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # 话题名称
        self.gt_topic = rospy.get_param('~ground_truth_topic', '/gazebo/model_states')
        self.mdetector_topic = rospy.get_param('~mdetector_topic', '/dynamic_points')
        self.fapp_topic = rospy.get_param('~fapp_topic', '/states')
        self.lvdot_topic = rospy.get_param('~lvdot_topic', '/onboard_detector/dynamic_bboxes')
        self.algo4_topic = rospy.get_param('~algo4_topic', '/algo4/detections')
        
        # 可视化开关
        self.enable_visualization = rospy.get_param('~enable_visualization', True)
        
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
        
        # Algorithm4
        self.algo4_sub = rospy.Subscriber(self.algo4_topic, PointCloud2, 
                                         self.algo4_callback, queue_size=10)
    
    def setup_publishers(self):
        """设置发布器"""
        if self.enable_visualization:
            self.gt_marker_pub = rospy.Publisher('/evaluation/ground_truth', MarkerArray, queue_size=10)
            self.det_marker_pubs = {
                'M-detector': rospy.Publisher('/evaluation/mdetector_detections', MarkerArray, queue_size=10),
                'FAPP': rospy.Publisher('/evaluation/fapp_detections', MarkerArray, queue_size=10),
                'LV-DOT': rospy.Publisher('/evaluation/lvdot_detections', MarkerArray, queue_size=10),
                'Algorithm4': rospy.Publisher('/evaluation/algo4_detections', MarkerArray, queue_size=10)
            }
            self.match_line_pub = rospy.Publisher('/evaluation/match_lines', Marker, queue_size=10)
            self.metrics_text_pub = rospy.Publisher('/evaluation/metrics_text', MarkerArray, queue_size=10)
    
    def gt_callback(self, msg):
        """Gazebo真值回调"""
        detections = []
        
        for i, name in enumerate(msg.name):
            # 过滤动态物体
            is_dynamic = any(keyword in name.lower() for keyword in self.dynamic_keywords)
            
            if is_dynamic:
                position = np.array([
                    msg.pose[i].position.x,
                    msg.pose[i].position.y,
                    msg.pose[i].position.z
                ])
                
                velocity = np.array([
                    msg.twist[i].linear.x,
                    msg.twist[i].linear.y,
                    msg.twist[i].linear.z
                ])
                
                # 估计边界框尺寸
                bbox_size = np.array(self.default_bbox_size)
                
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
    
    def algo4_callback(self, msg):
        """Algorithm4回调 - 统一使用点云解析"""
        detections = self.adapter.parse_mdetector(msg)  # 复用点云解析逻辑
        self.detection_buffers['Algorithm4'].append(detections)
    
    def evaluate_callback(self, event):
        """评估定时回调"""
        if len(self.current_gt) == 0:
            return
        
        current_time = rospy.Time.now().to_sec()
        
        # 评估每个算法
        for algo_name in ['M-detector', 'FAPP', 'LV-DOT', 'Algorithm4']:
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
            if self.enable_visualization:
                self.visualize_detections(algo_name, detections, matches)
        
        # 可视化真值
        if self.enable_visualization:
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
        
        # 颜色映射
        colors = {
            'M-detector': (0.9, 0.3, 0.3, 0.6),  # 红色
            'FAPP': (0.2, 0.6, 0.9, 0.6),        # 蓝色
            'LV-DOT': (0.2, 0.8, 0.4, 0.6),      # 绿色
            'Algorithm4': (0.95, 0.6, 0.1, 0.6)  # 橙色
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
        """可视化实时指标"""
        marker_array = MarkerArray()
        
        y_offset = 0.0
        for algo_name in ['M-detector', 'FAPP', 'LV-DOT', 'Algorithm4']:
            if len(self.algo_metrics_dict[algo_name]['metrics']) == 0:
                continue
            
            latest_metrics = self.algo_metrics_dict[algo_name]['metrics'][-1]
            
            text = f"{algo_name}:\n"
            text += f"Recall: {latest_metrics['recall']:.3f}\n"
            text += f"Precision: {latest_metrics['precision']:.3f}\n"
            text += f"F1: {latest_metrics['f1']:.3f}\n"
            text += f"MOTP: {latest_metrics['motp']:.3f}m"
            
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "metrics_text"
            marker.id = len(marker_array.markers)
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 3.0 + y_offset
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.3
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            
            marker.text = text
            marker.lifetime = rospy.Duration(0.5)
            
            marker_array.markers.append(marker)
            y_offset += 1.5
        
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
            plot_gen.plot_time_series(self.algo_metrics_dict)
        except Exception as e:
            rospy.logerr(f"Failed to generate time series plot: {e}")
        
        try:
            plot_gen.plot_radar_chart(overall_metrics)
        except Exception as e:
            rospy.logerr(f"Failed to generate radar chart: {e}")
        
        try:
            plot_gen.generate_performance_table(overall_metrics)
        except Exception as e:
            rospy.logerr(f"Failed to generate performance table: {e}")
        
        rospy.loginfo(f"Results saved to: {self.output_dir}")
        
        # 打印总结
        self.print_summary(overall_metrics)
    
    def save_frame_metrics_csv(self):
        """保存逐帧指标到CSV"""
        csv_path = os.path.join(self.output_dir, 'frame_metrics.csv')
        
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Timestamp', 'Algorithm', 'TP', 'FP', 'FN', 'Recall', 'Precision', 'F1', 'MOTP'])
            
            for algo_name in ['M-detector', 'FAPP', 'LV-DOT', 'Algorithm4']:
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
        json_path = os.path.join(self.output_dir, 'summary.json')
        
        summary = {
            'evaluation_time': rospy.Time.now().to_sec(),
            'duration': (rospy.Time.now() - self.start_time).to_sec(),
            'parameters': {
                'distance_threshold': self.distance_threshold,
                'iou_threshold': self.iou_threshold,
                'update_freq': self.update_freq
            },
            'algorithms': overall_metrics
        }
        
        with open(json_path, 'w') as f:
            json.dump(summary, f, indent=2)
    
    def print_summary(self, overall_metrics):
        """打印评估总结"""
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("EVALUATION SUMMARY")
        rospy.loginfo("="*80)
        
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
