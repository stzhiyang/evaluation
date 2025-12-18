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
from nav_msgs.msg import Odometry
import sensor_msgs.point_cloud2 as pc2
from sklearn.cluster import DBSCAN
import tf.transformations as tf_trans


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
        
        # 是否启用质心到几何中心的补偿
        self.enable_centroid_compensation = rospy.get_param('~enable_centroid_compensation', True)
        
        # 机体到激光雷达的外参（固定变换）
        # [x, y, z, roll, pitch, yaw] 激光雷达相对于机体坐标系的位置和姿态
        lidar_extrinsics = rospy.get_param('~lidar_extrinsics', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.lidar_translation = np.array(lidar_extrinsics[:3])
        self.lidar_rotation = np.array(lidar_extrinsics[3:])  # roll, pitch, yaw
        
        # 里程计话题
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        
        # 当前机器人位置和姿态（全局坐标系）
        self.robot_position = np.array([0.0, 0.0, 0.0])
        self.robot_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion [x, y, z, w]
        self.sensor_position = np.array([0.0, 0.0, 0.0])  # 传感器在全局坐标系中的位置
        
        # 订阅里程计
        if self.enable_centroid_compensation:
            self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self._odom_callback, queue_size=1)
        
        rospy.loginfo(f"DetectionAdapter initialized")
        rospy.loginfo(f"  DBSCAN eps: {self.dbscan_eps}, min_samples: {self.dbscan_min_samples}")
        rospy.loginfo(f"  Bbox size: dynamically computed from point cloud")
        rospy.loginfo(f"  Centroid compensation: {self.enable_centroid_compensation}")
        if self.enable_centroid_compensation:
            rospy.loginfo(f"  Odom topic: {self.odom_topic}")
            rospy.loginfo(f"  Lidar extrinsics: translation={self.lidar_translation}, rotation={self.lidar_rotation}")
        
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
            
            # 计算点云质心
            centroid = np.mean(cluster_points, axis=0)
            
            # 计算边界框尺寸 - 直接从点云计算，不使用固定最小值
            min_bound = np.min(cluster_points, axis=0)
            max_bound = np.max(cluster_points, axis=0)
            size = max_bound - min_bound
            
            # 防止尺寸为零（单点或共线点）
            size = np.maximum(size, 0.01)
            
            # 质心补偿：将质心转换为几何中心
            # 由于激光雷达只能看到物体的一面，质心会偏向传感器方向
            # 需要沿着远离传感器的方向偏移，估算物体的真实几何中心
            if self.enable_centroid_compensation:
                center = self._compensate_centroid_to_geometric_center(
                    centroid, cluster_points, size
                )
            else:
                center = centroid
            
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
    

    def _odom_callback(self, msg):
        """
        里程计回调，更新机器人位置和传感器位置
        Args:
            msg: nav_msgs/Odometry
        """
        # 更新机器人位置
        self.robot_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        # 更新机器人姿态（四元数）
        self.robot_orientation = np.array([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ])
        
        # 计算传感器在全局坐标系中的位置
        self.sensor_position = self._transform_lidar_to_world()
    
    def _transform_lidar_to_world(self):
        """
        将激光雷达坐标系转换到全局坐标系
        Returns:
            传感器在全局坐标系中的位置 [x, y, z]
        """
        # 将机器人姿态四元数转换为旋转矩阵
        rotation_matrix = tf_trans.quaternion_matrix(self.robot_orientation)[:3, :3]
        
        # 将激光雷达相对位置转换到全局坐标系
        # sensor_world = robot_position + R * lidar_translation
        sensor_position_world = self.robot_position + np.dot(rotation_matrix, self.lidar_translation)
        
        return sensor_position_world
    
    def _compensate_centroid_to_geometric_center(self, centroid, cluster_points, bbox_size):
        """
        将点云质心补偿为物体几何中心
        
        原理：
        - 激光雷达只能看到物体朝向传感器的表面
        - 点云质心会在 3D 空间中偏向传感器方向（比真实中心更近）
        - 需要在 3D 空间中沿着远离传感器的方向偏移
        
        方法：
        - 计算从传感器到质心的 3D 方向向量
        - 分析点云在该方向上的深度分布
        - 假设物体对称，将质心偏移到深度范围的中点
        
        Args:
            centroid: 点云质心 [x, y, z]
            cluster_points: 聚类点云 (N, 3)
            bbox_size: 边界框尺寸 [length, width, height]
        Returns:
            补偿后的几何中心 [x, y, z]
        """
        # 计算从传感器到质心的 3D 方向向量
        sensor_to_centroid = centroid - self.sensor_position
        
        distance_to_sensor = np.linalg.norm(sensor_to_centroid)
        if distance_to_sensor < 0.1:
            return centroid
        
        # 归一化方向向量（3D）
        direction = sensor_to_centroid / distance_to_sensor
        
        # 计算点云在传感器方向上的 3D 投影范围
        projections = np.dot(cluster_points - self.sensor_position, direction)
        min_proj = np.min(projections)
        max_proj = np.max(projections)
        
        # 补偿策略：假设物体是对称的，几何中心应该在深度范围的中点
        centroid_proj = np.dot(centroid - self.sensor_position, direction)
        ideal_proj = (min_proj + max_proj) / 2.0
        offset_distance = ideal_proj - centroid_proj
        
        # 应用 3D 补偿
        compensated_center = centroid + direction * offset_distance
        
        return compensated_center



if __name__ == '__main__':
    # 测试代码
    adapter = DetectionAdapter()
    print("DetectionAdapter initialized successfully")
