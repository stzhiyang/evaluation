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
    
    def __init__(self, distance_threshold=2.0, iou_threshold=0.1):
        """
        Args:
            distance_threshold: 最大匹配距离(米)
            iou_threshold: 最小IoU阈值
        """
        self.distance_threshold = distance_threshold
        self.iou_threshold = iou_threshold
    
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
        代价 = w1 * distance + w2 * (1 - IoU)
        """
        n_gt = len(ground_truth_list)
        n_det = len(detection_list)
        cost_matrix = np.zeros((n_gt, n_det))
        
        w_distance = 0.7
        w_iou = 0.3
        
        for i, gt in enumerate(ground_truth_list):
            for j, det in enumerate(detection_list):
                # 计算欧氏距离
                distance = self._compute_distance(gt, det)
                
                # 计算3D IoU
                iou = self._compute_3d_iou(gt, det)
                
                # 如果距离太远或IoU太小,设置为无穷大
                if distance > self.distance_threshold or iou < self.iou_threshold:
                    cost_matrix[i, j] = 1e6
                else:
                    cost_matrix[i, j] = w_distance * distance + w_iou * (1.0 - iou)
        
        return cost_matrix
    
    def _compute_distance(self, obj1, obj2):
        """计算两个物体中心的欧氏距离"""
        return np.linalg.norm(obj1.position - obj2.position)
    
    def _compute_3d_iou(self, obj1, obj2):
        """
        计算3D边界框IoU
        假设边界框与坐标轴对齐(AABB)
        """
        # 边界框的最小和最大坐标
        min1 = obj1.position - obj1.bbox_size / 2.0
        max1 = obj1.position + obj1.bbox_size / 2.0
        
        min2 = obj2.position - obj2.bbox_size / 2.0
        max2 = obj2.position + obj2.bbox_size / 2.0
        
        # 计算交集
        inter_min = np.maximum(min1, min2)
        inter_max = np.minimum(max1, max2)
        
        # 检查是否有交集
        if np.any(inter_min >= inter_max):
            return 0.0
        
        inter_volume = np.prod(inter_max - inter_min)
        
        # 计算并集
        volume1 = np.prod(obj1.bbox_size)
        volume2 = np.prod(obj2.bbox_size)
        union_volume = volume1 + volume2 - inter_volume
        
        # 避免除零
        if union_volume < 1e-6:
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
