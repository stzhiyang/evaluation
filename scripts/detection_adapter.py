#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Detection Adapter Module
将不同算法的检测输出转换为统一格式
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import MarkerArray

# FAPP消息类型
try:
    from obj_state_msgs.msg import ObjectsStates, State
    FAPP_AVAILABLE = True
except ImportError:
    rospy.logwarn("obj_state_msgs not found, FAPP support disabled")
    FAPP_AVAILABLE = False


class StandardDetection:
    """标准检测结果格式"""
    def __init__(self, obj_id, position, velocity, bbox_size, timestamp, point_cloud=None):
        self.id = obj_id
        self.position = np.array(position)  # [x, y, z]
        self.velocity = np.array(velocity)  # [vx, vy, vz]
        self.bbox_size = np.array(bbox_size)  # [length, width, height]
        self.timestamp = timestamp
        self.point_cloud = point_cloud  # np.array of shape (N, 3) for IOU calculation
    
    def to_dict(self):
        return {
            'id': self.id,
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'bbox_size': self.bbox_size.tolist(),
            'timestamp': self.timestamp,
            'num_points': len(self.point_cloud) if self.point_cloud is not None else 0
        }


class DetectionAdapter:
    """检测结果适配器"""
    
    def __init__(self):
        # DBSCAN聚类参数（仅用于M-detector点云）
        self.dbscan_eps = rospy.get_param('~mdetector_dbscan_eps', 0.8)
        self.dbscan_min_samples = rospy.get_param('~mdetector_dbscan_min_samples', 5)
        
        rospy.loginfo(f"DetectionAdapter initialized")
        rospy.loginfo(f"  M-detector: DBSCAN eps={self.dbscan_eps}, min_samples={self.dbscan_min_samples}")
        rospy.loginfo(f"  FAPP: ObjectsStates with position/velocity/size")
        rospy.loginfo(f"  LV-DOT/LDOT: MarkerArray with bbox position/size")
        
    def parse_mdetector(self, pointcloud_msg):
        """
        解析M-detector的点云输出
        支持两种模式:
        1. /m_detector/point_out - 原始动态点,需要DBSCAN聚类
        2. /m_detector/frame_out - 已聚类点云,直接提取簇
        Args:
            pointcloud_msg: sensor_msgs/PointCloud2
        Returns:
            list of StandardDetection
        """
        if pointcloud_msg is None:
            return []
        
        # 提取点云数据
        points = []
        for p in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([p[0], p[1], p[2]])
        
        if len(points) == 0:
            return []
        
        points = np.array(points)
        
        # DBSCAN聚类
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_
        
        # 为每个簇创建检测结果
        detections = []
        unique_labels = set(labels)
        unique_labels.discard(-1)  # 移除噪声点
        
        for label in unique_labels:
            cluster_points = points[labels == label]
            
            # 计算点云质心和边界框尺寸
            center = np.mean(cluster_points, axis=0)
            min_bound = np.min(cluster_points, axis=0)
            max_bound = np.max(cluster_points, axis=0)
            size = max_bound - min_bound
            size = np.maximum(size, 0.01)  # 防止尺寸为零
            
            # 速度信息（点云不提供）
            velocity = np.array([0.0, 0.0, 0.0])
            
            detection = StandardDetection(
                obj_id=int(label),
                position=center,
                velocity=velocity,
                bbox_size=size,
                timestamp=pointcloud_msg.header.stamp.to_sec(),
                point_cloud=cluster_points  # 保存点云数据用于IOU计算
            )
            detections.append(detection)
        
        return detections
    
    def parse_fapp(self, objects_states_msg):
        """
        解析FAPP的ObjectsStates输出
        Args:
            objects_states_msg: obj_state_msgs/ObjectsStates
        Returns:
            list of StandardDetection
        """
        if objects_states_msg is None or not FAPP_AVAILABLE:
            return []
        
        detections = []
        
        for i, state in enumerate(objects_states_msg.states):
            position = np.array([
                state.position.x,
                state.position.y,
                state.position.z
            ])
            
            velocity = np.array([
                state.velocity.x,
                state.velocity.y,
                state.velocity.z
            ])
            
            bbox_size = np.array([
                state.size.x,
                state.size.y,
                state.size.z
            ])
            
            # FAPP的size字段可能为0，使用默认尺寸
            if np.sum(bbox_size) < 0.01:  # 如果尺寸太小或为0
                bbox_size = np.array([0.5, 0.5, 1.5])  # 默认人体尺寸
            
            # FAPP有独立的位置输出，点云为None
            detection = StandardDetection(
                obj_id=i,
                position=position,
                velocity=velocity,
                bbox_size=bbox_size,
                timestamp=objects_states_msg.header.stamp.to_sec(),
                point_cloud=None  # FAPP无点云，有独立位置
            )
            detections.append(detection)
        
        return detections
    
    def parse_marker_array(self, marker_array_msg):
        """
        解析LV-DOT/LDOT的MarkerArray输出（动态bbox）
        Args:
            marker_array_msg: visualization_msgs/MarkerArray
        Returns:
            list of StandardDetection
        """
        if marker_array_msg is None:
            return []
        
        detections = []
        
        for marker in marker_array_msg.markers:
            if marker.type != 5:  # LINE_LIST type
                continue
            
            # 从marker提取位置和尺寸
            # marker.pose.position是box中心位置
            # marker中的corner点定义了box尺寸
            position = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])
            
            # 从corner点计算尺寸
            # corner[0] = (-x_width/2, -y_width/2, -z_width)
            # 根据publish3dBox的实现，第一个点是corner[0]
            if len(marker.points) >= 2:
                # 第一条边是corner[0]到corner[1]
                corner0 = marker.points[0]
                corner2 = marker.points[4]  # corner[2]在第3条边
                
                # 计算实际尺寸（corner在局部坐标系）
                x_width = abs(corner2.x - corner0.x)
                y_width = abs(corner2.y - corner0.y)
                z_width = abs(corner0.z) * 2  # corner0.z = -z_width
                
                bbox_size = np.array([x_width, y_width, z_width])
            else:
                # 默认尺寸
                bbox_size = np.array([0.5, 0.5, 1.5])
            
            # 速度信息（MarkerArray不提供）
            velocity = np.array([0.0, 0.0, 0.0])
            
            # LV-DOT/LDOT有独立的位置输出
            detection = StandardDetection(
                obj_id=marker.id,
                position=position,
                velocity=velocity,
                bbox_size=bbox_size,
                timestamp=marker.header.stamp.to_sec() if marker.header.stamp.to_sec() > 0 else rospy.Time.now().to_sec(),
                point_cloud=None  # 有独立位置，不需要点云
            )
            detections.append(detection)
        
        return detections


if __name__ == '__main__':
    # 测试代码
    adapter = DetectionAdapter()
    print("DetectionAdapter initialized successfully")
