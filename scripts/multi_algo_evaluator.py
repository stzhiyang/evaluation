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
from geometry_msgs.msg import Point, PoseArray, PoseStamped
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
        self.adapter = DetectionAdapter(mocap_id_height_map=self.mocap_id_height_map)
        self.associator = DataAssociator(
            distance_threshold=self.distance_threshold,
            iou_threshold=self.iou_threshold,
            mocap_mode=self.mocap_mode
        )
        
        # 为每个算法创建计算器
        self.calculators = {
            'M-detector': MetricsCalculator(),
            'FAPP': MetricsCalculator(),
            'LV-DOT': MetricsCalculator(),
            'LDOT': MetricsCalculator()
        }
        
        # 数据缓冲区
        self.gt_buffer = deque(maxlen=200)  # 增加到200，约20秒@10Hz
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
        
        # 多物体动捕模式相关变量
        self.mocap_subscribers = {}  # 存储动捕订阅器 {topic_name: subscriber}
        self.mocap_poses = {}  # 存储接收到的动捕姿态 {object_id: PoseStamped}
        self.mocap_object_types = {}  # 存储物体类型映射 {object_id: object_type}
        
        # 节点关闭标志
        self.is_shutdown = False
        
        # 发布器 - 必须在订阅器之前创建，避免回调中使用未初始化的发布器
        self.setup_publishers()
        
        # 订阅器 - 创建后会立即开始接收消息
        self.setup_subscribers()
        
        # 定时器
        self.eval_timer = rospy.Timer(rospy.Duration(1.0 / self.update_freq), self.evaluate_callback)
        
        # 启动时间
        self.start_time = rospy.Time.now()
        
        rospy.loginfo("Multi-Algorithm Evaluator initialized")
        rospy.loginfo(f"Ground truth source: {self.gt_source}")
        if self.gt_source == 'mocap':
            rospy.loginfo(f"Mocap mode enabled: 3D distance matching (no IOU)")
            rospy.loginfo(f"Mocap ID-height mapping: {self.mocap_id_height_map}")
            rospy.loginfo(f"Mocap object classes: {self.mocap_object_classes}")
        rospy.loginfo(f"Algorithms: M-detector, FAPP, LV-DOT, LDOT")
        rospy.loginfo(f"Update frequency: {self.update_freq} Hz")
        rospy.loginfo(f"Visualization mode: {self.visualization_mode}")
        rospy.loginfo(f"Timestamp mode: {self.timestamp_mode}")
        rospy.loginfo(f"Matching criteria:")
        if self.mocap_mode:
            rospy.loginfo(f"  - 3D distance < {self.distance_threshold}m (mocap mode, no IOU)")
        else:
            rospy.loginfo(f"  - 3D Bounding Box IOU >= {self.iou_threshold} AND")
            rospy.loginfo(f"  - Euclidean distance < {self.distance_threshold}m")
        rospy.loginfo(f"  - Timestamp sync mode: {self.timestamp_sync_mode}")
        if self.timestamp_sync_mode == 'tolerance':
            rospy.loginfo(f"  - Timestamp tolerance: {self.timestamp_tolerance*1000:.0f}ms (nearest neighbor matching)")
        else:
            rospy.loginfo(f"  - Using latest detection and latest ground truth")
        rospy.loginfo(f"  - Position: point cloud centroid (all algorithms)")
        rospy.loginfo(f"  - Bbox size: from point cloud min/max bounds")
        rospy.loginfo(f"Evaluation metrics explanation:")
        rospy.loginfo(f"  - MOTP (visualization): Current frame average position error")
        rospy.loginfo(f"  - MOTP (final result): All frames cumulative average position error")
        rospy.loginfo(f"Results will be saved to: {self.output_dir}")
        
        # 打印评估控制信息
        if self.max_frames > 0:
            rospy.loginfo(f"Max evaluation frames: {self.max_frames}")
    
    def load_parameters(self):
        """加载参数"""
        # 匹配阈值
        self.distance_threshold = rospy.get_param('~distance_threshold', 1.0)  # 欧氏距离阈值
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.1)  # 点云IOU阈值
        
        # 动捕模式：True时只使用XY平面距离关联，忽略Z轴
        self.mocap_mode = rospy.get_param('~mocap_mode', False)
        
        # 动捕ID到Z轴高度的映射（仅在mocap_mode=True时使用）
        # 格式：{'car': 0.5, 'person': 1.0, 'drone': 1.5}
        self.mocap_id_height_map = rospy.get_param('~mocap_id_height_map', {})
        
        # 动捕物体类别列表（用于提供默认类别映射，实际类别从话题名称提取）
        # 例如：['car', 'person', 'drone']
        self.mocap_object_classes = rospy.get_param('~mocap_object_classes', [])

        # 动捕物体ID到类别名称的映射（用于把具体ID归并到配置中的类别名）
        # 例如：{'p1': 'person', 'p2': 'person', 'UAV_1': 'uav'}
        self.mocap_object_id_to_class_map = rospy.get_param('~mocap_object_id_to_class_map', {})
        
        # 更新频率
        self.update_freq = rospy.get_param('~update_freq', 10.0)
        
        # 时间戳同步模式：'tolerance' 或 'latest'
        self.timestamp_sync_mode = rospy.get_param('~timestamp_sync_mode', 'tolerance')
        
        # 时间戳同步容差（秒）- 仅在 tolerance 模式下使用
        # 检测结果和真值的时间戳差异超过此值则不进行匹配
        # 对于10Hz点云，建议设置为 0.15（150ms，略大于一帧间隔100ms）
        self.timestamp_tolerance = rospy.get_param('~timestamp_tolerance', 0.15)
        
        # 可视化模式：'immediate' 在接收到数据时立即可视化（推荐），'timer' 在定时器中可视化
        self.visualization_mode = rospy.get_param('~visualization_mode', 'immediate')
        
        # 时间戳模式：'original' 使用消息原始时间戳（推荐，保留时序准确性），'now' 使用当前时间戳（实时显示）
        self.timestamp_mode = rospy.get_param('~timestamp_mode', 'original')
        
        # 是否打印时间戳调试信息
        self.debug_timestamps = rospy.get_param('~debug_timestamps', False)
        
        # 评估控制参数 - 新增
        self.max_frames = rospy.get_param('~max_frames', 0)  # 最大评估帧数，0表示无限制
        self.frame_count = 0  # 当前已评估帧数
        
        # 输出目录 - 创建带时间戳的子文件夹
        base_output_dir = rospy.get_param('~output_dir', 
                                         os.path.join(os.path.dirname(__file__), '../results/algorithm_eval'))
        
        # 如果是相对路径，转换为相对于evaluation包根目录的绝对路径
        if not os.path.isabs(base_output_dir):
            base_output_dir = os.path.join(os.path.dirname(__file__), '..', base_output_dir)
        
        # 生成时间戳
        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # 创建带eval_前缀的时间戳子文件夹
        self.output_dir = os.path.join(base_output_dir, f"eval_{self.timestamp}")
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # 话题名称
        self.gt_topic = rospy.get_param('~ground_truth_topic', '/gazebo/model_states')
        self.gt_source = rospy.get_param('~ground_truth_source', 'gazebo')  # 'gazebo' 或 'mocap'
        self.mdetector_topic = rospy.get_param('~mdetector_topic', '/dynamic_points')
        self.fapp_topic = rospy.get_param('~fapp_topic', '/states')
        self.lvdot_topic = rospy.get_param('~lvdot_topic', '/onboard_detector/dynamic_bboxes')
        self.ldot_topic = rospy.get_param('~ldot_topic', '/ldot_detector/dynamic_point_cloud')
        
        # Gazebo动态物体过滤关键词（仅当gt_source='gazebo'时使用）
        self.dynamic_keywords = rospy.get_param('~dynamic_keywords', ['actor', 'dynamic', 'person', 'obstacle'])
        
        # 物体尺寸估计(根据Gazebo模型类型)
        self.default_bbox_size = rospy.get_param('~default_bbox_size', [0.5, 0.5, 1.8])
    
    def setup_mocap_multi_subscribers(self):
        """设置多物体动捕模式的订阅器"""
        rospy.loginfo("Setting up multi-object mocap subscribers...")

        # 查找所有 /vrpn_client_node/*/pose 话题
        mocap_topics = []
        try:
            # 获取所有已发布话题：返回 list of [topic_name, topic_type]
            all_topics = rospy.get_published_topics()

            for topic_name, topic_type in all_topics:
                # 检查是否符合动捕话题模式
                if (topic_name.startswith(self.gt_topic + '/') and 
                    topic_name.endswith('/pose') and
                    topic_type == 'geometry_msgs/PoseStamped'):
                    mocap_topics.append(topic_name)
                    
        except Exception as e:
            rospy.logerr(f"Failed to discover mocap topics: {e}")
            return
        
        if not mocap_topics:
            rospy.logwarn(f"No mocap topics found matching pattern {self.gt_topic}/*/pose")
            return
        
        rospy.loginfo(f"Found {len(mocap_topics)} mocap topics:")
        for topic in mocap_topics:
            rospy.loginfo(f"  - {topic}")
            
            # 从话题名称提取物体ID和类型
            # 例如：/vrpn_client_node/UAV_3/pose -> object_id="UAV_3", object_type="UAV"
            topic_parts = topic.split('/')
            if len(topic_parts) >= 3:
                object_id = topic_parts[2]  # UAV_3

                # 先生成默认类型，再统一转小写，保证与配置中的 key（如 uav/person/car）一致
                object_type = object_id.split('_')[0] if '_' in object_id else object_id
                object_type = object_type.lower()

                # 应用 object_id -> 类别名 的映射（优先级最高）
                mapped_type = self.mocap_object_id_to_class_map.get(object_id)
                if isinstance(mapped_type, str) and mapped_type:
                    object_type = mapped_type.lower()
                
                # 存储物体类型映射
                self.mocap_object_types[object_id] = object_type
                
                # 创建订阅器
                subscriber = rospy.Subscriber(topic, PoseStamped, 
                                            lambda msg, oid=object_id: self.mocap_single_pose_callback(msg, oid), 
                                            queue_size=10)
                self.mocap_subscribers[topic] = subscriber
                
                rospy.loginfo(f"  Subscribed to {topic} -> {object_id} (type: {object_type})")
        
        rospy.loginfo(f"Multi-object mocap setup complete. Tracking {len(self.mocap_subscribers)} objects.")
    
    def mocap_single_pose_callback(self, pose_stamped, object_id):
        """单个物体的动捕姿态回调"""
        if self.is_shutdown:
            return
            
        # 存储接收到的姿态
        self.mocap_poses[object_id] = pose_stamped
        
        # 检查是否所有物体都已更新（基于时间戳）
        if len(self.mocap_poses) > 0:
            # 使用adapter解析多个动捕姿态
            detections = self.adapter.parse_mocap_multi(
                self.mocap_poses, self.mocap_object_types, self.mocap_id_height_map
            )
            
            self.current_gt = detections
            self.gt_buffer.append(detections)
            
            # 在 immediate 模式下立即可视化真值
            if self.visualization_mode == 'immediate':
                self.visualize_ground_truth()
    
    def setup_subscribers(self):
        """设置订阅器"""
        # 真值订阅 - 动捕模式
        if self.gt_source == 'mocap':
            rospy.loginfo(f"Using Mocap ground truth from: {self.gt_topic}")
            # 多物体模式：动态发现并订阅多个PoseStamped话题
            self.setup_mocap_multi_subscribers()
        else:  # gazebo
            rospy.loginfo(f"Using Gazebo ground truth from: {self.gt_topic}")
            self.gt_sub = rospy.Subscriber(self.gt_topic, ModelStates, self.gt_callback, queue_size=10)
        
        # M-detector: 动态点云
        self.mdetector_sub = rospy.Subscriber(self.mdetector_topic, PointCloud2, 
                                             self.mdetector_callback, queue_size=10)
        
        # FAPP: ObjectsStates包含位置、速度、尺寸
        try:
            from obj_state_msgs.msg import ObjectsStates
            self.fapp_sub = rospy.Subscriber(self.fapp_topic, ObjectsStates, 
                                            self.fapp_callback, queue_size=10)
        except ImportError:
            rospy.logwarn("obj_state_msgs not found, FAPP subscription disabled")
        
        # LV-DOT: MarkerArray包含动态bbox（位置+尺寸）
        self.lvdot_sub = rospy.Subscriber(self.lvdot_topic, MarkerArray, 
                                         self.lvdot_callback, queue_size=10)
        
        # LDOT: MarkerArray包含动态bbox（位置+尺寸）
        self.ldot_sub = rospy.Subscriber(self.ldot_topic, MarkerArray, 
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
                
                # 真值直接使用边界框信息，无需生成点云
                # IOU计算使用3D边界框IOU（position + bbox_size）
                # 使用仿真时间作为时间戳（rospy.Time.now() 在 use_sim_time=true 时返回仿真时间）
                detection = StandardDetection(
                    obj_id=i,
                    position=position,
                    velocity=velocity,
                    bbox_size=bbox_size,
                    timestamp=rospy.Time.now().to_sec(),  # Gazebo ModelStates 无 header，使用当前仿真时间
                    point_cloud=None  # 真值不需要点云
                )
                detections.append(detection)
        
        self.current_gt = detections
        self.gt_buffer.append(detections)
        
        # 在 immediate 模式下立即可视化真值，保持与检测结果同步
        if self.visualization_mode == 'immediate':
            self.visualize_ground_truth()
    
    def mocap_gt_callback(self, msg):
        """动捕真值回调"""
        # 使用adapter解析动捕数据
        detections = self.adapter.parse_mocap(msg, self.mocap_object_classes)
        
        self.current_gt = detections
        self.gt_buffer.append(detections)
        
        # 在 immediate 模式下立即可视化真值
        if self.visualization_mode == 'immediate':
            self.visualize_ground_truth()
    
    def mdetector_callback(self, msg):
        """M-detector回调 - 点云解析"""
        if self.is_shutdown:
            return
        detections = self.adapter.parse_mdetector(msg)
        self.detection_buffers['M-detector'].append(detections)
        
        # 立即可视化检测结果
        if self.visualization_mode == 'immediate':
            self.visualize_detections('M-detector', detections, [])
    
    def fapp_callback(self, msg):
        """FAPP回调 - ObjectsStates解析"""
        if self.is_shutdown:
            return
        detections = self.adapter.parse_fapp(msg)
        self.detection_buffers['FAPP'].append(detections)
        
        # 立即可视化检测结果
        if self.visualization_mode == 'immediate':
            self.visualize_detections('FAPP', detections, [])
    
    def lvdot_callback(self, msg):
        """LV-DOT回调 - MarkerArray解析"""
        if self.is_shutdown:
            return
        detections = self.adapter.parse_marker_array(msg)
        self.detection_buffers['LV-DOT'].append(detections)
        
        # 立即可视化检测结果
        if self.visualization_mode == 'immediate':
            self.visualize_detections('LV-DOT', detections, [])
    
    def ldot_callback(self, msg):
        """LDOT回调 - MarkerArray解析"""
        if self.is_shutdown:
            return
        detections = self.adapter.parse_marker_array(msg)
        self.detection_buffers['LDOT'].append(detections)
        
        # 立即可视化检测结果
        if self.visualization_mode == 'immediate':
            self.visualize_detections('LDOT', detections, [])
    
    def find_nearest_gt(self, detection_timestamp):
        """
        根据检测结果的时间戳，在真值缓冲区中找到时间最接近且不晚于检测结果的真值帧
        
        Args:
            detection_timestamp: 检测结果的时间戳（秒）
        Returns:
            (best_gt, min_time_diff): 最接近的真值列表和时间差（秒）
            如果缓冲区为空或超过容差则返回 (None, None)
        """
        if len(self.gt_buffer) == 0:
            return None, None
        
        best_gt = None
        min_time_diff = float('inf')
        
        # 创建快照避免并发修改问题
        gt_buffer_snapshot = list(self.gt_buffer)
        
        for gt_list in gt_buffer_snapshot:
            if len(gt_list) == 0:
                continue
            
            gt_timestamp = gt_list[0].timestamp
            
            # 时间顺序约束：真值时间戳不能晚于检测结果时间戳
            if gt_timestamp > detection_timestamp:
                continue
            
            time_diff = abs(gt_timestamp - detection_timestamp)
            
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                best_gt = gt_list
        
        # 如果时间差超过阈值，认为没有有效匹配
        if min_time_diff > self.timestamp_tolerance:
            if self.debug_timestamps:
                rospy.logwarn_throttle(2.0, f"No GT within tolerance: min_diff={min_time_diff*1000:.1f}ms > {self.timestamp_tolerance*1000:.1f}ms")
            return None, min_time_diff
        
        return best_gt, min_time_diff
    
    def get_latest_gt(self):
        """
        获取最新的真值数据（用于 latest 模式）
        
        Returns:
            latest_gt: 最新的真值列表，如果缓冲区为空则返回 None
        """
        if len(self.gt_buffer) == 0:
            return None
        
        # 返回缓冲区中最新的真值
        return self.gt_buffer[-1]
    
    def evaluate_callback(self, event):
        """评估定时回调"""
        if len(self.gt_buffer) == 0:
            return
        
        current_time = rospy.Time.now().to_sec()
        
        # 收集所有算法的时间同步信息
        sync_info = {}
        
        # 评估每个算法
        for algo_name in ['M-detector', 'FAPP', 'LV-DOT', 'LDOT']:
            if len(self.detection_buffers[algo_name]) == 0:
                continue
            
            # 获取最新检测结果
            detections = self.detection_buffers[algo_name][-1]
            
            if len(detections) == 0:
                continue
            
            # 根据时间戳同步模式选择真值匹配方式
            if self.timestamp_sync_mode == 'latest':
                # latest 模式：直接使用最新的真值
                matched_gt = self.get_latest_gt()
                
                if matched_gt is None:
                    sync_info[algo_name] = "NO_GT"
                    continue
                
                # 记录时间同步信息（显示时间差但不作为过滤条件）
                detection_timestamp = detections[0].timestamp
                gt_timestamp = matched_gt[0].timestamp if len(matched_gt) > 0 else 0
                time_diff = abs(detection_timestamp - gt_timestamp)
                sync_info[algo_name] = f"LATEST({time_diff*1000:.1f}ms)"
                
            else:  # tolerance 模式（默认）
                # 根据检测结果的时间戳找到最接近的真值帧
                detection_timestamp = detections[0].timestamp
                matched_gt, time_diff = self.find_nearest_gt(detection_timestamp)
                
                if matched_gt is None:
                    # 没有找到时间匹配的真值，跳过本次评估
                    sync_info[algo_name] = "NO_GT"
                    continue
                
                # 记录时间同步信息
                sync_info[algo_name] = f"{time_diff*1000:.1f}ms"
            
            # 数据关联 - 使用时间对齐后的真值
            matches, unmatched_gt, unmatched_det = self.associator.associate(
                matched_gt, detections
            )
            
            # 计算指标
            metrics = self.calculators[algo_name].compute_frame_metrics(
                matches, unmatched_gt, unmatched_det
            )
            
            # 记录
            self.algo_metrics_dict[algo_name]['timestamps'].append(current_time)
            self.algo_metrics_dict[algo_name]['metrics'].append(metrics)
            
            # 在定时器模式下可视化检测结果
            if self.visualization_mode == 'timer':
                self.visualize_detections(algo_name, detections, matches)
        
        # 打印所有算法的时间同步状态（每1秒一次），有数据才会打印
        if len(sync_info) > 0:
            sync_str = " | ".join([f"{k}:{v}" for k, v in sync_info.items()])
            rospy.loginfo_throttle(1.0, f"[Time Sync] {sync_str}")
        
        # 增加帧计数
        self.frame_count += 1
        
        # 检查是否达到评估限制（在评估完当前帧之后）
        if self.should_stop_evaluation():
            rospy.loginfo("Evaluation limit reached. Stopping...")
            self.save_results()
            self.is_shutdown = True  # 设置关闭标志
            rospy.signal_shutdown("Evaluation completed")
            return
        
        # 在 timer 模式下可视化真值和指标
        if self.visualization_mode == 'timer':
            self.visualize_ground_truth()
        
        # 指标文本始终在定时器中更新（避免过于频繁）
        self.visualize_metrics()
    
    def visualize_ground_truth(self):
        """可视化真值"""
        marker_array = MarkerArray()
        
        # 根据时间戳模式选择时间戳
        if self.timestamp_mode == 'original' and len(self.current_gt) > 0 and self.current_gt[0].timestamp is not None:
            # 使用真值数据的原始时间戳，保留时序准确性
            timestamp = rospy.Time.from_sec(self.current_gt[0].timestamp)
        else:
            # 使用 Time(0) 让 RViz 自动使用最新的 TF 变换
            timestamp = rospy.Time(0)
        
        for detection in self.current_gt:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = timestamp  # 使用 Time(0) 确保实时显示
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
        
        # 检查节点是否正在关闭
        if not self.is_shutdown:
            try:
                self.gt_marker_pub.publish(marker_array)
            except rospy.ROSException:
                pass
    
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
        
        # 根据时间戳模式选择时间戳
        if self.timestamp_mode == 'original' and len(detections) > 0 and detections[0].timestamp is not None:
            # 使用检测数据的原始时间戳，可以看出算法延迟
            timestamp = rospy.Time.from_sec(detections[0].timestamp)
        else:
            # 使用 Time(0) 让 RViz 自动使用最新的 TF 变换
            timestamp = rospy.Time(0)
        
        for detection in detections:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = timestamp  # 使用 Time(0) 确保实时显示
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
        
        # 检查节点是否正在关闭，避免发布到已关闭的 topic
        if not self.is_shutdown and algo_name in self.det_marker_pubs:
            try:
                self.det_marker_pubs[algo_name].publish(marker_array)
            except rospy.ROSException:
                # 忽略关闭时的发布错误
                pass
    
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
            
            # 算法名称和指标文本 - 显示更详细的信息
            text = f"{algo_name}\n"
            text += f"R:{latest_metrics['recall']:.2f} "
            text += f"P:{latest_metrics['precision']:.2f}\n"
            text += f"F1:{latest_metrics['f1']:.2f}\n"
            text += f"MOTP:{latest_metrics['motp']:.2f}m\n"
            text += f"TP:{latest_metrics['tp']} "
            text += f"FP:{latest_metrics['fp']} "
            text += f"FN:{latest_metrics['fn']}"
            
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time(0)  # 使用 Time(0) 确保文本显示
            marker.ns = "metrics_text"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # 横向排列位置
            marker.pose.position.x = x_offset + i * x_spacing
            marker.pose.position.y = 0.0
            marker.pose.position.z = 4.0  # 固定高度
            marker.pose.orientation.w = 1.0
            
            marker.scale.z = 0.35  # 字体大小（调小以容纳更多文本）
            
            # 使用算法对应的颜色
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = 1.0
            
            marker.text = text
            marker.lifetime = rospy.Duration(0.5)
            
            marker_array.markers.append(marker)
        
        # 检查节点是否正在关闭
        if not self.is_shutdown:
            try:
                self.metrics_text_pub.publish(marker_array)
            except rospy.ROSException:
                pass
    
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
        
        # 生成图表 - 只生成性能表格（PNG和TXT格式）
        plot_gen = PlotGenerator(self.output_dir)
        
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
            if not self.is_shutdown:
                self.save_results()


if __name__ == '__main__':
    try:
        evaluator = MultiAlgorithmEvaluator()
        evaluator.run()
    except rospy.ROSInterruptException:
        pass
