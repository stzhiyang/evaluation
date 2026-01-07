#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Data Association Module
实现基于匈牙利算法的数据关联和评估指标计算
"""

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.distance import cdist


class DataAssociator:
    """数据关联器"""
    
    def __init__(self, distance_threshold=2.0, iou_threshold=0.1, mocap_mode=False):
        """
        Args:
            distance_threshold: 最大匹配距离(米) - 用于有位置输出的算法
            iou_threshold: 最小3D边界框IOU阈值 - 用于Gazebo模式
            mocap_mode: 动捕模式，为True时只使用XY平面距离进行关联，不使用IOU
        """
        self.distance_threshold = distance_threshold
        self.iou_threshold = iou_threshold
        self.mocap_mode = mocap_mode
    
    def associate(self, ground_truth_list, detection_list):
        """
        使用匈牙利算法关联真值和检测结果
        Args:
            ground_truth_list: 真值对象列表 (StandardDetection)
            detection_list: 检测结果列表 (StandardDetection)
        Returns:
            matches: list of (gt_idx, det_idx, distance)
            unmatched_gt: list of gt_idx
            unmatched_det: list of det_idx
        """
        if len(ground_truth_list) == 0 and len(detection_list) == 0:
            return [], [], []
        
        if len(ground_truth_list) == 0:
            return [], [], list(range(len(detection_list)))
        
        if len(detection_list) == 0:
            return [], list(range(len(ground_truth_list))), []
        
        # 构建代价矩阵
        cost_matrix = self._build_cost_matrix(ground_truth_list, detection_list)
        
        # 匈牙利算法求解
        gt_indices, det_indices = linear_sum_assignment(cost_matrix)
        
        # 筛选有效匹配
        matches = []
        matched_gt = set()
        matched_det = set()
        
        for gt_idx, det_idx in zip(gt_indices, det_indices):
            cost = cost_matrix[gt_idx, det_idx]
            distance = self._compute_distance(
                ground_truth_list[gt_idx],
                detection_list[det_idx]
            )
            
            # 检查是否满足阈值条件
            if distance < self.distance_threshold and cost < 1e6:
                matches.append((gt_idx, det_idx, distance))
                matched_gt.add(gt_idx)
                matched_det.add(det_idx)
        
        # 未匹配的真值和检测
        unmatched_gt = [i for i in range(len(ground_truth_list)) if i not in matched_gt]
        unmatched_det = [i for i in range(len(detection_list)) if i not in matched_det]
        
        return matches, unmatched_gt, unmatched_det
    
    def _build_cost_matrix(self, ground_truth_list, detection_list):
        """
        构建代价矩阵
        匹配条件：
        - Gazebo模式：
          1. 3D边界框IOU >= iou_threshold
          2. 3D欧氏距离 < distance_threshold
        - 动捕模式：
          只使用XY平面距离 < distance_threshold（不使用IOU）
        
        边界框从点云计算：
        - position: 点云质心
        - bbox_size: 点云的min/max范围
        """
        n_gt = len(ground_truth_list)
        n_det = len(detection_list)
        cost_matrix = np.zeros((n_gt, n_det))
        
        for i, gt in enumerate(ground_truth_list):
            for j, det in enumerate(detection_list):
                # 计算距离（动捕模式使用XY距离，Gazebo模式使用3D距离）
                distance = self._compute_distance(gt, det)
                
                if self.mocap_mode:
                    # 动捕模式：只使用距离判断
                    if distance < self.distance_threshold:
                        cost_matrix[i, j] = distance
                    else:
                        cost_matrix[i, j] = 1e6
                else:
                    # Gazebo模式：距离+IOU双重条件
                    iou = self._compute_bbox_iou(gt, det)
                    if iou >= self.iou_threshold and distance < self.distance_threshold:
                        # 代价：距离和(1-IOU)的加权组合
                        cost_matrix[i, j] = distance + (1.0 - iou) * 2.0
                    else:
                        # 无效匹配设置为大值
                        cost_matrix[i, j] = 1e6
        
        return cost_matrix
    
    def _compute_distance(self, obj1, obj2):
        """
        计算两个物体中心的3D欧氏距离
        动捕模式和Gazebo模式都使用完整的3D距离
        """
        # 统一使用3D欧氏距离
        return np.linalg.norm(obj1.position - obj2.position)
    
    def _compute_bbox_iou(self, obj1, obj2):
        """
        计算两个边界框的3D IOU（仅用于Gazebo模式）
        动捕模式不使用此函数
        
        Args:
            obj1, obj2: StandardDetection对象，包含position和bbox_size属性
        Returns:
            iou: 交并比 [0, 1]
        """
        # 计算每个边界框的最小和最大坐标
        half_size1 = obj1.bbox_size / 2.0
        half_size2 = obj2.bbox_size / 2.0
        
        min1 = obj1.position - half_size1
        max1 = obj1.position + half_size1
        min2 = obj2.position - half_size2
        max2 = obj2.position + half_size2
        
        # 计算交集边界框
        inter_min = np.maximum(min1, min2)
        inter_max = np.minimum(max1, max2)
        
        # 检查是否有交集
        if np.any(inter_min >= inter_max):
            return 0.0
        
        # 计算交集体积
        inter_size = inter_max - inter_min
        inter_volume = np.prod(inter_size)
        
        # 计算并集体积
        volume1 = np.prod(obj1.bbox_size)
        volume2 = np.prod(obj2.bbox_size)
        union_volume = volume1 + volume2 - inter_volume
        
        if union_volume == 0:
            return 0.0
        
        iou = inter_volume / union_volume
        return iou


class MetricsCalculator:
    """评估指标计算器"""
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        """重置累积统计"""
        self.total_tp = 0
        self.total_fp = 0
        self.total_fn = 0
        self.total_distance_error = 0.0
        self.total_matched = 0
        
        # 逐帧记录
        self.frame_metrics = []
    
    def compute_frame_metrics(self, matches, unmatched_gt, unmatched_det):
        """
        计算单帧的评估指标
        Args:
            matches: list of (gt_idx, det_idx, distance)
            unmatched_gt: list of gt_idx
            unmatched_det: list of det_idx
        Returns:
            dict with keys: tp, fp, fn, recall, precision, f1, motp
        """
        tp = len(matches)
        fp = len(unmatched_det)
        fn = len(unmatched_gt)
        
        # 计算MOTP(平均定位误差)
        if tp > 0:
            motp = np.mean([distance for _, _, distance in matches])
        else:
            motp = 0.0
        
        # 计算召回率、精准率、F1
        recall = tp / (tp + fn) if (tp + fn) > 0 else 0.0
        precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
        
        metrics = {
            'tp': tp,
            'fp': fp,
            'fn': fn,
            'recall': recall,
            'precision': precision,
            'f1': f1,
            'motp': motp
        }
        
        # 累积统计
        self.total_tp += tp
        self.total_fp += fp
        self.total_fn += fn
        if tp > 0:
            self.total_distance_error += sum([distance for _, _, distance in matches])
            self.total_matched += tp
        
        self.frame_metrics.append(metrics)
        
        return metrics
    
    def compute_overall_metrics(self):
        """
        计算整体评估指标
        Returns:
            dict with keys: recall, precision, f1, motp, mean, std
        """
        # 基于累积统计计算
        recall = self.total_tp / (self.total_tp + self.total_fn) if (self.total_tp + self.total_fn) > 0 else 0.0
        precision = self.total_tp / (self.total_tp + self.total_fp) if (self.total_tp + self.total_fp) > 0 else 0.0
        f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
        motp = self.total_distance_error / self.total_matched if self.total_matched > 0 else 0.0
        
        # 计算逐帧指标的均值和标准差
        if len(self.frame_metrics) > 0:
            recalls = [m['recall'] for m in self.frame_metrics]
            precisions = [m['precision'] for m in self.frame_metrics]
            f1s = [m['f1'] for m in self.frame_metrics]
            motps = [m['motp'] for m in self.frame_metrics if m['motp'] > 0]
            
            metrics = {
                'recall_mean': np.mean(recalls),
                'recall_std': np.std(recalls),
                'precision_mean': np.mean(precisions),
                'precision_std': np.std(precisions),
                'f1_mean': np.mean(f1s),
                'f1_std': np.std(f1s),
                'motp_mean': np.mean(motps) if len(motps) > 0 else 0.0,
                'motp_std': np.std(motps) if len(motps) > 0 else 0.0,
                'total_tp': self.total_tp,
                'total_fp': self.total_fp,
                'total_fn': self.total_fn
            }
        else:
            metrics = {
                'recall_mean': 0.0,
                'recall_std': 0.0,
                'precision_mean': 0.0,
                'precision_std': 0.0,
                'f1_mean': 0.0,
                'f1_std': 0.0,
                'motp_mean': 0.0,
                'motp_std': 0.0,
                'total_tp': 0,
                'total_fp': 0,
                'total_fn': 0
            }
        
        return metrics


if __name__ == '__main__':
    # 测试代码
    associator = DataAssociator()
    calculator = MetricsCalculator()
    print("DataAssociator and MetricsCalculator initialized successfully")
