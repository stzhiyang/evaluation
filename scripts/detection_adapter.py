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
        
        # 位置估计方法: 'centroid'(质心) 或 'bbox'(边界框中心) 或 'obb'(定向边界框)
        self.position_method = rospy.get_param('~position_estimation_method', 'bbox')
        
        rospy.loginfo(f"DetectionAdapter initialized")
        rospy.loginfo(f"  DBSCAN eps: {self.dbscan_eps}, min_samples: {self.dbscan_min_samples}")
        rospy.loginfo(f"  Position estimation: {self.position_method}")
        rospy.loginfo(f"  Bbox size: dynamically computed from point cloud")
        
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
            
            # 计算物体位置和边界框
            center, size = self._estimate_object_center_and_size(cluster_points)
            
            # 速度信息（点云不提供）
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
    
    def _estimate_object_center_and_size(self, points):
        """
        估计物体的中心位置和尺寸
        解决激光雷达只能看到物体一侧导致质心偏移的问题
        
        Args:
            points: Nx3 numpy array of point cloud
        
        Returns:
            center: 3D position (estimated object center)
            size: 3D bounding box size
        """
        if self.position_method == 'centroid':
            # 方法1: 简单质心（存在偏移问题）
            center = np.mean(points, axis=0)
            min_bound = np.min(points, axis=0)
            max_bound = np.max(points, axis=0)
            size = max_bound - min_bound
            
        elif self.position_method == 'bbox':
            # 方法2: 轴对齐边界框(AABB)中心（推荐，简单有效）
            min_bound = np.min(points, axis=0)
            max_bound = np.max(points, axis=0)
            center = (min_bound + max_bound) / 2.0  # 使用边界框几何中心
            size = max_bound - min_bound
            
        elif self.position_method == 'obb':
            # 方法3: 定向边界框(OBB)中心（最准确，使用PCA找主方向）
            center, size = self._compute_oriented_bbox(points)
            
        else:
            rospy.logwarn(f"Unknown position method: {self.position_method}, using bbox")
            min_bound = np.min(points, axis=0)
            max_bound = np.max(points, axis=0)
            center = (min_bound + max_bound) / 2.0
            size = max_bound - min_bound
        
        # 防止尺寸为零
        size = np.maximum(size, 0.01)
        
        return center, size
    
    def _compute_oriented_bbox(self, points):
        """
        使用PCA计算定向边界框(OBB)
        通过主成分分析找到物体的主方向，在主方向坐标系中计算边界框
        
        Args:
            points: Nx3 numpy array
        
        Returns:
            center: OBB中心位置
            size: OBB尺寸
        """
        # 计算质心
        centroid = np.mean(points, axis=0)
        
        # 中心化点云
        centered_points = points - centroid
        
        # PCA - 找到主方向
        cov_matrix = np.cov(centered_points.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)
        
        # 按特征值排序（从大到小）
        idx = eigenvalues.argsort()[::-1]
        eigenvectors = eigenvectors[:, idx]
        
        # 将点云转换到主方向坐标系
        transformed_points = centered_points @ eigenvectors
        
        # 在主方向坐标系中计算AABB
        min_bound = np.min(transformed_points, axis=0)
        max_bound = np.max(transformed_points, axis=0)
        
        # OBB中心（在主方向坐标系中）
        obb_center_local = (min_bound + max_bound) / 2.0
        
        # 转换回世界坐标系
        obb_center_world = centroid + (eigenvectors @ obb_center_local)
        
        # OBB尺寸
        size = max_bound - min_bound
        
        return obb_center_world, size


if __name__ == '__main__':
    # 测试代码
    adapter = DetectionAdapter()
    print("DetectionAdapter initialized successfully")
