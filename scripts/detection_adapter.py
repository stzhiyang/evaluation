#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Detection Adapter Module
将不同算法的检测输出转换为统一格式
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Vector3
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN


class StandardDetection:
    """标准检测结果格式"""
    def __init__(self, obj_id, position, velocity, bbox_size, timestamp):
        self.id = obj_id
        self.position = np.array(position)  # [x, y, z]
        self.velocity = np.array(velocity)  # [vx, vy, vz]
        self.bbox_size = np.array(bbox_size)  # [length, width, height]
        self.timestamp = timestamp
    
    def to_dict(self):
        return {
            'id': self.id,
            'position': self.position.tolist(),
            'velocity': self.velocity.tolist(),
            'bbox_size': self.bbox_size.tolist(),
            'timestamp': self.timestamp
        }


class DetectionAdapter:
    """检测结果适配器"""
    
    def __init__(self):
        # M-detector DBSCAN聚类参数
        self.dbscan_eps = rospy.get_param('~mdetector_dbscan_eps', 0.8)
        self.dbscan_min_samples = rospy.get_param('~mdetector_dbscan_min_samples', 5)
        
        # 最小边界框尺寸 - 防止点云稀疏导致box过小
        default_size = rospy.get_param('~default_bbox_size', [0.5, 0.5, 1.8])
        self.min_bbox_size = np.array(default_size)
        
        rospy.loginfo(f"DetectionAdapter initialized")
        rospy.loginfo(f"  DBSCAN eps: {self.dbscan_eps}, min_samples: {self.dbscan_min_samples}")
        rospy.loginfo(f"  Min bbox size: {self.min_bbox_size}")
        
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
        # 注意: 如果输入已经是frame_out(聚类后),DBSCAN仍会工作
        # 因为聚类后的点云各簇之间距离较远,eps参数能正确分离
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_
        
        # 为每个簇创建检测结果
        detections = []
        unique_labels = set(labels)
        unique_labels.discard(-1)  # 移除噪声点
        
        for label in unique_labels:
            cluster_points = points[labels == label]
            
            # 使用质心作为物体中心位置
            # 质心 = 点云的均值，相比几何中心（AABB中心）更能反映点云的实际分布
            # 注意：由于雷达只能看到物体的一面，质心仍会偏向物体表面
            # 但相比几何中心，质心受点云密度影响，通常更接近真实中心
            center = np.mean(cluster_points, axis=0)
            
            # 计算边界框尺寸
            min_bound = np.min(cluster_points, axis=0)
            max_bound = np.max(cluster_points, axis=0)
            size = max_bound - min_bound
            
            # 应用最小边界框尺寸限制 - 点云通常只覆盖物体一面，需要扩展到合理尺寸
            size = np.maximum(size, self.min_bbox_size)
            
            # M-detector不提供速度信息
            velocity = np.array([0.0, 0.0, 0.0])
            
            detection = StandardDetection(
                obj_id=int(label),
                position=center,
                velocity=velocity,
                bbox_size=size,
                timestamp=pointcloud_msg.header.stamp.to_sec()
            )
            detections.append(detection)
        
        return detections
    
    def parse_fapp(self, states_msg):
        """
        解析FAPP的ObjectsStates消息
        Args:
            states_msg: obj_state_msgs/ObjectsStates
        Returns:
            list of StandardDetection
        """
        if states_msg is None:
            return []
        
        detections = []
        for i, state in enumerate(states_msg.states):
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
            
            detection = StandardDetection(
                obj_id=i,
                position=position,
                velocity=velocity,
                bbox_size=bbox_size,
                timestamp=states_msg.header.stamp.to_sec()
            )
            detections.append(detection)
        
        return detections
    
    def parse_lvdot(self, marker_array_msg):
        """
        解析LV-DOT的MarkerArray输出
        Args:
            marker_array_msg: visualization_msgs/MarkerArray
        Returns:
            list of StandardDetection
        """
        if marker_array_msg is None:
            return []
        
        detections = []
        for marker in marker_array_msg.markers:
            position = np.array([
                marker.pose.position.x,
                marker.pose.position.y,
                marker.pose.position.z
            ])
            
            bbox_size = np.array([
                marker.scale.x,
                marker.scale.y,
                marker.scale.z
            ])
            
            # 尝试从marker文本中提取速度信息
            velocity = self._extract_velocity_from_marker(marker)
            
            detection = StandardDetection(
                obj_id=marker.id,
                position=position,
                velocity=velocity,
                bbox_size=bbox_size,
                timestamp=rospy.Time.now().to_sec()  # MarkerArray可能没有时间戳
            )
            detections.append(detection)
        
        return detections
    
    def parse_generic_markerarray(self, marker_array_msg):
        """
        解析通用MarkerArray格式(第四算法)
        Args:
            marker_array_msg: visualization_msgs/MarkerArray
        Returns:
            list of StandardDetection
        """
        # 默认使用与LV-DOT相同的解析方式
        return self.parse_lvdot(marker_array_msg)
    
    def _extract_velocity_from_marker(self, marker):
        """
        从Marker的text字段提取速度信息(如果有)
        Args:
            marker: visualization_msgs/Marker
        Returns:
            numpy array [vx, vy, vz]
        """
        # 默认速度为0
        velocity = np.array([0.0, 0.0, 0.0])
        
        if hasattr(marker, 'text') and marker.text:
            try:
                # 尝试解析类似 "vel: 1.2, 0.5, 0.0" 的格式
                if 'vel:' in marker.text.lower():
                    vel_str = marker.text.lower().split('vel:')[1].strip()
                    vel_parts = vel_str.split(',')
                    if len(vel_parts) >= 3:
                        velocity = np.array([
                            float(vel_parts[0]),
                            float(vel_parts[1]),
                            float(vel_parts[2])
                        ])
            except:
                pass
        
        return velocity


if __name__ == '__main__':
    # 测试代码
    adapter = DetectionAdapter()
    print("DetectionAdapter initialized successfully")
