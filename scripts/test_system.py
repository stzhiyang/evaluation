#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试评估系统各模块是否正常工作
"""

import sys
import os

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

def test_imports():
    """测试导入"""
    print("="*60)
    print("Testing imports...")
    print("="*60)
    
    try:
        import numpy as np
        print("✅ numpy")
    except ImportError as e:
        print(f"❌ numpy: {e}")
        return False
    
    try:
        from scipy.optimize import linear_sum_assignment
        print("✅ scipy")
    except ImportError as e:
        print(f"❌ scipy: {e}")
        return False
    
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        print("✅ matplotlib")
    except ImportError as e:
        print(f"❌ matplotlib: {e}")
        return False
    
    try:
        from sklearn.cluster import DBSCAN
        print("✅ scikit-learn")
    except ImportError as e:
        print(f"❌ scikit-learn: {e}")
        return False
    
    return True

def test_modules():
    """测试自定义模块"""
    print("\n" + "="*60)
    print("Testing custom modules...")
    print("="*60)
    
    try:
        from detection_adapter import DetectionAdapter, StandardDetection
        adapter = DetectionAdapter()
        print("✅ DetectionAdapter")
    except Exception as e:
        print(f"❌ DetectionAdapter: {e}")
        return False
    
    try:
        from data_association import DataAssociator, MetricsCalculator
        associator = DataAssociator()
        calculator = MetricsCalculator()
        print("✅ DataAssociator & MetricsCalculator")
    except Exception as e:
        print(f"❌ DataAssociator & MetricsCalculator: {e}")
        return False
    
    try:
        from plot_generator import PlotGenerator
        generator = PlotGenerator()
        print("✅ PlotGenerator")
    except Exception as e:
        print(f"❌ PlotGenerator: {e}")
        return False
    
    return True

def test_functionality():
    """测试基本功能"""
    print("\n" + "="*60)
    print("Testing functionality...")
    print("="*60)
    
    import numpy as np
    from detection_adapter import StandardDetection
    from data_association import DataAssociator, MetricsCalculator
    
    # 创建测试数据
    gt1 = StandardDetection(0, [1.0, 2.0, 0.5], [0.1, 0.2, 0.0], [0.5, 0.5, 1.5], 0.0)
    gt2 = StandardDetection(1, [3.0, 4.0, 0.5], [0.2, 0.1, 0.0], [0.5, 0.5, 1.5], 0.0)
    
    det1 = StandardDetection(0, [1.1, 2.1, 0.5], [0.15, 0.25, 0.0], [0.6, 0.6, 1.6], 0.0)
    det2 = StandardDetection(1, [3.2, 4.1, 0.5], [0.25, 0.15, 0.0], [0.55, 0.55, 1.55], 0.0)
    det3 = StandardDetection(2, [10.0, 10.0, 0.5], [0.0, 0.0, 0.0], [0.5, 0.5, 1.5], 0.0)
    
    ground_truth = [gt1, gt2]
    detections = [det1, det2, det3]
    
    # 测试数据关联
    associator = DataAssociator(distance_threshold=2.0, iou_threshold=0.1)
    matches, unmatched_gt, unmatched_det = associator.associate(ground_truth, detections)
    
    print(f"Matches: {len(matches)}")
    print(f"Unmatched GT: {len(unmatched_gt)}")
    print(f"Unmatched Detections: {len(unmatched_det)}")
    
    if len(matches) != 2:
        print("❌ Expected 2 matches")
        return False
    
    if len(unmatched_det) != 1:
        print("❌ Expected 1 unmatched detection")
        return False
    
    print("✅ Data association works correctly")
    
    # 测试指标计算
    calculator = MetricsCalculator()
    metrics = calculator.compute_frame_metrics(matches, unmatched_gt, unmatched_det)
    
    print(f"TP: {metrics['tp']}, FP: {metrics['fp']}, FN: {metrics['fn']}")
    print(f"Recall: {metrics['recall']:.3f}")
    print(f"Precision: {metrics['precision']:.3f}")
    print(f"F1: {metrics['f1']:.3f}")
    print(f"MOTP: {metrics['motp']:.3f}")
    
    if metrics['tp'] != 2 or metrics['fp'] != 1 or metrics['fn'] != 0:
        print("❌ Metrics calculation incorrect")
        return False
    
    print("✅ Metrics calculation works correctly")
    
    return True

def main():
    print("\n" + "="*60)
    print("Evaluation System Test Suite")
    print("="*60 + "\n")
    
    success = True
    
    # 测试导入
    if not test_imports():
        print("\n❌ Import test failed!")
        print("\nPlease install missing dependencies:")
        print("  pip3 install numpy scipy matplotlib scikit-learn")
        success = False
    
    # 测试模块
    if success and not test_modules():
        print("\n❌ Module test failed!")
        success = False
    
    # 测试功能
    if success and not test_functionality():
        print("\n❌ Functionality test failed!")
        success = False
    
    # 总结
    print("\n" + "="*60)
    if success:
        print("✅ All tests passed!")
        print("="*60)
        print("\nThe evaluation system is ready to use.")
        print("Run: roslaunch evaluation evaluation_gazebo.launch")
    else:
        print("❌ Some tests failed!")
        print("="*60)
        print("\nPlease fix the issues above before using the system.")
    print("")
    
    return 0 if success else 1

if __name__ == '__main__':
    sys.exit(main())
