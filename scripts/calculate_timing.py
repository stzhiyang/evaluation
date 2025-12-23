#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
耗时计算脚本
读取指定的耗时记录 CSV 文件，计算平均耗时
"""

import rospy
import os
import csv
from datetime import datetime
from pathlib import Path


def is_valid_row(row, algorithm_name, exclude_columns=['Timestamp', 'PointCount']):
    """
    检查一行数据是否有效
    有效条件：
    - LV-DOT: 只需要 TotalTime > 1
    - 其他算法: 除了 Timestamp 和 PointCount 外，所有数值列的值都要大于 0.001
    
    Args:
        row: 字典格式的一行数据
        algorithm_name: 算法名称
        exclude_columns: 需要排除检查的列名列表
    
    Returns:
        bool: 是否为有效数据
    """
    try:
        # LV-DOT 特殊处理：只检查 TotalTime > 1
        if algorithm_name in ['LV-DOT', 'LVDOT', 'LV_DOT']:
            if 'TotalTime' in row:
                return float(row['TotalTime']) > 1.0
            return False
        
        # 其他算法：所有时间列都要大于 0.001
        for key, value in row.items():
            # 跳过需要排除的列
            if key in exclude_columns:
                continue
            
            # 检查数值是否大于 0.001
            if float(value) <= 0.001:
                return False
        return True
    except (ValueError, TypeError):
        return False


def calculate_average_timing(csv_file_path, algorithm_name):
    """
    计算单个 CSV 文件的平均耗时
    
    Args:
        csv_file_path: CSV 文件路径
        algorithm_name: 算法名称
    
    Returns:
        dict: 包含计算结果的字典
    """
    result = {
        'algorithm': algorithm_name,
        'file': csv_file_path,
        'valid': False,
        'average_time': None,
        'valid_count': 0,
        'total_count': 0,
        'error': None
    }
    
    if not os.path.exists(csv_file_path):
        result['error'] = f"文件不存在: {csv_file_path}"
        rospy.logwarn(result['error'])
        return result
    
    valid_rows = []
    total_rows = 0
    
    try:
        with open(csv_file_path, 'r') as f:
            reader = csv.DictReader(f)
            
            # 检查是否有 TotalTime 列
            if 'TotalTime' not in reader.fieldnames:
                result['error'] = "文件缺少 TotalTime 列"
                rospy.logwarn(f"{algorithm_name}: {result['error']}")
                return result
            
            # 读取所有有效数据
            for row in reader:
                total_rows += 1
                if is_valid_row(row, algorithm_name):
                    valid_rows.append(float(row['TotalTime']))
        
        result['total_count'] = total_rows
        result['valid_count'] = len(valid_rows)
        
        # 检查是否满足至少 100 组有效数据的条件
        if len(valid_rows) < 100:
            result['error'] = f"有效数据不足 100 组 (当前: {len(valid_rows)})"
            rospy.logwarn(f"{algorithm_name}: {result['error']}")
            return result
        
        # 取倒数 100 组有效数据计算平均值
        last_100 = valid_rows[-100:]
        result['average_time'] = sum(last_100) / len(last_100)
        result['valid'] = True
        
        rospy.loginfo(f"{algorithm_name}: 平均耗时 = {result['average_time']:.6f} 秒 (有效数据: {result['valid_count']})")
        
    except Exception as e:
        result['error'] = f"读取文件出错: {str(e)}"
        rospy.logerr(f"{algorithm_name}: {result['error']}")
    
    return result


def generate_report(results, output_dir):
    """
    生成耗时计算报告
    
    Args:
        results: 计算结果列表
        output_dir: 输出目录
    
    Returns:
        str: 报告文件路径
    """
    # 创建输出目录
    os.makedirs(output_dir, exist_ok=True)
    
    # 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    report_file = os.path.join(output_dir, f"timing_report_{timestamp}.txt")
    
    # 生成报告内容
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write("=" * 80 + "\n")
        f.write("算法耗时计算报告\n")
        f.write("=" * 80 + "\n\n")
        
        f.write(f"计算时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"计算条件: 至少 100 组有效数据（所有时间变量 > 0.001 秒）\n")
        f.write(f"计算方法: 取倒数 100 组有效数据的 TotalTime 平均值\n\n")
        
        f.write("=" * 80 + "\n")
        f.write("计算结果\n")
        f.write("=" * 80 + "\n\n")
        
        # 统计成功和失败的数量
        success_count = sum(1 for r in results if r['valid'])
        
        for result in results:
            f.write(f"算法: {result['algorithm']}\n")
            f.write(f"文件: {result['file']}\n")
            
            if result['valid']:
                f.write(f"状态: ✓ 计算成功\n")
                f.write(f"平均耗时: {result['average_time']:.6f} 秒\n")
                f.write(f"有效数据: {result['valid_count']} 组\n")
                f.write(f"总数据量: {result['total_count']} 组\n")
            else:
                f.write(f"状态: ✗ 计算失败\n")
                f.write(f"错误信息: {result['error']}\n")
                if result['valid_count'] > 0:
                    f.write(f"有效数据: {result['valid_count']} 组 (需要至少 100 组)\n")
                    f.write(f"总数据量: {result['total_count']} 组\n")
            
            f.write("\n" + "-" * 80 + "\n\n")
        
        f.write("=" * 80 + "\n")
        f.write("汇总统计\n")
        f.write("=" * 80 + "\n\n")
        
        f.write(f"总算法数: {len(results)}\n")
        f.write(f"成功计算: {success_count}\n")
        f.write(f"计算失败: {len(results) - success_count}\n\n")
        
        # 如果有成功的结果，显示对比
        if success_count > 0:
            f.write("成功算法耗时对比:\n")
            for result in results:
                if result['valid']:
                    f.write(f"  {result['algorithm']:20s}: {result['average_time']:.6f} 秒\n")
        
        f.write("\n" + "=" * 80 + "\n")
    
    return report_file


def main():
    """
    主函数：从 ROS 参数读取配置并计算耗时
    """
    rospy.init_node('timing_calculator', anonymous=False)
    
    rospy.loginfo("=" * 80)
    rospy.loginfo("算法耗时计算节点启动")
    rospy.loginfo("=" * 80)
    
    # 从 ROS 参数服务器读取配置
    csv_files = rospy.get_param('~csv_files', {})
    output_dir = rospy.get_param('~output_dir', '')
    
    if not csv_files:
        rospy.logerr("未配置 CSV 文件！请在 launch 文件中设置 csv_files 参数")
        return
    
    if not output_dir:
        rospy.logerr("未配置输出目录！请在 launch 文件中设置 output_dir 参数")
        return
    
    rospy.loginfo(f"输出目录: {output_dir}")
    rospy.loginfo(f"配置的算法数量: {len(csv_files)}")
    rospy.loginfo("")
    
    # 计算每个算法的耗时
    results = []
    for algorithm_name, csv_path in csv_files.items():
        rospy.loginfo(f"处理算法: {algorithm_name}")
        rospy.loginfo(f"CSV 文件: {csv_path}")
        
        result = calculate_average_timing(csv_path, algorithm_name)
        results.append(result)
        rospy.loginfo("")
    
    # 生成报告
    rospy.loginfo("生成计算报告...")
    report_file = generate_report(results, output_dir)
    
    rospy.loginfo("=" * 80)
    rospy.loginfo(f"报告已生成: {report_file}")
    rospy.loginfo("=" * 80)
    
    # 在终端显示简要结果
    print("\n" + "=" * 80)
    print("计算完成！")
    print("=" * 80)
    for result in results:
        if result['valid']:
            print(f"{result['algorithm']:20s}: {result['average_time']:.6f} 秒")
        else:
            print(f"{result['algorithm']:20s}: 计算失败 - {result['error']}")
    print("=" * 80)
    print(f"详细报告: {report_file}")
    print("=" * 80 + "\n")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
