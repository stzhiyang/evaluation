#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Performance Evaluator Node
性能评估主节点 - 单算法性能消耗评估
"""

import rospy
import json
import os
import sys
from datetime import datetime
import psutil
import time

# 添加脚本目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)


class SingleAlgorithmPerformanceEvaluator:
    """单算法性能评估器 - 对比算法启动前后的CPU和内存消耗"""
    
    def __init__(self):
        rospy.init_node('performance_evaluator', anonymous=False)
        
        # 加载参数
        self.load_parameters()
        
        # 性能数据
        self.baseline_cpu = 0.0
        self.baseline_memory = 0.0
        self.running_cpu = 0.0
        self.running_memory = 0.0
        
        rospy.loginfo("="*50)
        rospy.loginfo("单算法性能评估器已初始化")
        rospy.loginfo("="*50)
        rospy.loginfo(f"算法名称: {self.algorithm_name}")
        rospy.loginfo(f"节点名称: {self.node_name}")
        rospy.loginfo(f"监控频率: {self.monitor_freq} Hz")
        rospy.loginfo(f"基线测量时长: {self.baseline_duration} 秒")
        rospy.loginfo(f"运行测量时长: {self.running_duration} 秒")
        rospy.loginfo(f"结果保存到: {self.output_dir}")
        rospy.loginfo("="*50)
    
    def load_parameters(self):
        """加载参数"""
        # 算法名称和节点名称
        self.algorithm_name = rospy.get_param('~algorithm_name', 'Algorithm')
        self.node_name = rospy.get_param('~node_name', '/algorithm_node')
        
        # 监控频率
        self.monitor_freq = rospy.get_param('~monitor_freq', 1.0)
        
        # 基线测量时长（秒）
        self.baseline_duration = rospy.get_param('~baseline_duration', 10.0)
        
        # 运行测量时长（秒）
        self.running_duration = rospy.get_param('~running_duration', 60.0)
        
        # 输出目录 - 使用绝对路径
        output_dir_param = rospy.get_param('~output_dir', 'results')
        
        # 如果是相对路径，则相对于 evaluation 包的路径
        if not os.path.isabs(output_dir_param):
            # 获取 evaluation 包的路径
            package_dir = os.path.abspath(os.path.join(script_dir, '..'))
            base_output_dir = os.path.join(package_dir, output_dir_param)
        else:
            base_output_dir = output_dir_param
        
        # 创建带时间戳和算法名称的子文件夹
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        safe_algo_name = self.algorithm_name.replace(' ', '_').replace('-', '_')
        self.output_dir = os.path.join(base_output_dir, 
                                      f'performance_{safe_algo_name}_{self.timestamp}')
        
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
    
    def measure_system_resources(self, duration, description):
        """
        测量系统资源占用
        Args:
            duration: 测量时长（秒）
            description: 描述信息
        Returns:
            (cpu_mean, memory_mean): CPU和内存的平均值
        """
        rospy.loginfo(f"正在测量{description}...")
        rospy.loginfo(f"测量时长: {duration} 秒")
        
        cpu_samples = []
        memory_samples = []
        
        start_time = time.time()
        sample_interval = 1.0 / self.monitor_freq
        
        while time.time() - start_time < duration:
            # 测量整体系统资源
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory_info = psutil.virtual_memory()
            memory_mb = memory_info.used / (1024 * 1024)
            
            cpu_samples.append(cpu_percent)
            memory_samples.append(memory_mb)
            
            # 显示进度
            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and len(cpu_samples) > 0:
                rospy.loginfo(f"  进度: {elapsed:.0f}/{duration:.0f}s - "
                            f"CPU: {cpu_samples[-1]:.1f}%, "
                            f"内存: {memory_samples[-1]:.0f}MB")
            
            time.sleep(sample_interval)
        
        cpu_mean = sum(cpu_samples) / len(cpu_samples) if cpu_samples else 0.0
        memory_mean = sum(memory_samples) / len(memory_samples) if memory_samples else 0.0
        
        rospy.loginfo(f"{description}测量完成:")
        rospy.loginfo(f"  平均 CPU: {cpu_mean:.2f}%")
        rospy.loginfo(f"  平均内存: {memory_mean:.2f} MB")
        rospy.loginfo(f"  采样数: {len(cpu_samples)}")
        
        return cpu_mean, memory_mean, cpu_samples, memory_samples
    
    def check_node_running(self):
        """检查节点是否在运行"""
        try:
            import subprocess
            result = subprocess.run(
                ['rosnode', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            return self.node_name in result.stdout
        except Exception as e:
            rospy.logwarn(f"无法检查节点状态: {e}")
            return False
    
    def run_evaluation(self):
        """运行完整的性能评估流程"""
        
        # ========== 步骤 1: 测量基线 ==========
        rospy.loginfo("="*50)
        rospy.loginfo("步骤 1: 测量系统基线（算法启动前）")
        rospy.loginfo("="*50)
        rospy.loginfo(f"请确保 {self.algorithm_name} 节点尚未启动！")
        rospy.loginfo("按 Enter 键开始测量基线...")
        
        try:
            input()  # 等待用户确认
        except:
            pass
        
        # 检查节点是否已经在运行
        if self.check_node_running():
            rospy.logwarn(f"警告: 检测到节点 {self.node_name} 已在运行！")
            rospy.logwarn("基线测量可能不准确，建议先停止节点。")
            rospy.loginfo("是否继续？(y/n)")
            try:
                response = input().strip().lower()
                if response != 'y':
                    rospy.loginfo("评估已取消")
                    return
            except:
                pass
        
        self.baseline_cpu, self.baseline_memory, baseline_cpu_samples, baseline_mem_samples = \
            self.measure_system_resources(self.baseline_duration, "系统基线")
        
        # ========== 步骤 2: 启动算法 ==========
        rospy.loginfo("="*50)
        rospy.loginfo("步骤 2: 启动算法节点")
        rospy.loginfo("="*50)
        rospy.loginfo(f"请现在启动 {self.algorithm_name} 节点")
        rospy.loginfo(f"节点名称应为: {self.node_name}")
        rospy.loginfo("启动后按 Enter 键继续...")
        
        try:
            input()  # 等待用户确认
        except:
            pass
        
        # 等待节点初始化
        rospy.loginfo("等待节点初始化（30秒）...")
        rospy.sleep(30.0)
        
        # 检查节点是否在运行
        if not self.check_node_running():
            rospy.logwarn(f"警告: 未检测到节点 {self.node_name}")
            rospy.logwarn("请确认节点名称是否正确")
            rospy.loginfo("是否继续测量？(y/n)")
            try:
                response = input().strip().lower()
                if response != 'y':
                    rospy.loginfo("评估已取消")
                    return
            except:
                pass
        else:
            rospy.loginfo(f"✓ 检测到节点 {self.node_name} 正在运行")
        
        # ========== 步骤 3: 测量运行状态 ==========
        rospy.loginfo("="*50)
        rospy.loginfo("步骤 3: 测量算法运行时的系统资源")
        rospy.loginfo("="*50)
        
        self.running_cpu, self.running_memory, running_cpu_samples, running_mem_samples = \
            self.measure_system_resources(self.running_duration, "算法运行状态")
        
        # ========== 步骤 4: 保存结果 ==========
        rospy.loginfo("="*50)
        rospy.loginfo("步骤 4: 保存评估结果")
        rospy.loginfo("="*50)
        
        self.save_results(baseline_cpu_samples, baseline_mem_samples,
                         running_cpu_samples, running_mem_samples)
        
        # 打印总结
        self.print_summary()
    
    def save_results(self, baseline_cpu_samples, baseline_mem_samples,
                    running_cpu_samples, running_mem_samples):
        """保存评估结果"""
        
        # 计算增量
        cpu_delta = self.running_cpu - self.baseline_cpu
        memory_delta = self.running_memory - self.baseline_memory
        
        # 计算标准差
        import numpy as np
        baseline_cpu_std = np.std(baseline_cpu_samples)
        baseline_mem_std = np.std(baseline_mem_samples)
        running_cpu_std = np.std(running_cpu_samples)
        running_mem_std = np.std(running_mem_samples)
        
        # 保存JSON统计
        json_path = os.path.join(self.output_dir, f'performance_result_{self.timestamp}.json')
        
        result = {
            'algorithm_name': self.algorithm_name,
            'node_name': self.node_name,
            'evaluation_time': datetime.now().isoformat(),
            'timestamp': self.timestamp,
            'baseline_duration': self.baseline_duration,
            'running_duration': self.running_duration,
            'monitor_freq': self.monitor_freq,
            'baseline': {
                'cpu_mean': round(self.baseline_cpu, 2),
                'cpu_std': round(baseline_cpu_std, 2),
                'memory_mean': round(self.baseline_memory, 2),
                'memory_std': round(baseline_mem_std, 2),
                'sample_count': len(baseline_cpu_samples)
            },
            'running': {
                'cpu_mean': round(self.running_cpu, 2),
                'cpu_std': round(running_cpu_std, 2),
                'memory_mean': round(self.running_memory, 2),
                'memory_std': round(running_mem_std, 2),
                'sample_count': len(running_cpu_samples)
            },
            'delta': {
                'cpu': round(cpu_delta, 2),
                'memory': round(memory_delta, 2),
                'cpu_percentage': round((cpu_delta / self.baseline_cpu * 100) if self.baseline_cpu > 0 else 0, 2),
                'memory_percentage': round((memory_delta / self.baseline_memory * 100) if self.baseline_memory > 0 else 0, 2)
            }
        }
        
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(result, f, indent=2, ensure_ascii=False)
        
        rospy.loginfo(f"✓ 保存统计数据: {json_path}")
        
        # 保存文本报告
        txt_path = os.path.join(self.output_dir, f'performance_report_{self.timestamp}.txt')
        
        with open(txt_path, 'w', encoding='utf-8') as f:
            f.write("="*50 + "\n")
            f.write(f"{self.algorithm_name} 性能评估报告\n")
            f.write("="*50 + "\n\n")
            
            f.write(f"评估时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"算法名称: {self.algorithm_name}\n")
            f.write(f"节点名称: {self.node_name}\n\n")
            
            f.write(f"基线测量时长: {self.baseline_duration} 秒\n")
            f.write(f"运行测量时长: {self.running_duration} 秒\n")
            f.write(f"监控频率: {self.monitor_freq} Hz\n\n")
            
            f.write("="*50 + "\n")
            f.write("测量结果\n")
            f.write("="*50 + "\n\n")
            
            f.write("系统基线（算法启动前）:\n")
            f.write(f"  CPU 使用率:  {self.baseline_cpu:.2f} ± {baseline_cpu_std:.2f} %\n")
            f.write(f"  内存使用:    {self.baseline_memory:.2f} ± {baseline_mem_std:.2f} MB\n\n")
            
            f.write("算法运行时:\n")
            f.write(f"  CPU 使用率:  {self.running_cpu:.2f} ± {running_cpu_std:.2f} %\n")
            f.write(f"  内存使用:    {self.running_memory:.2f} ± {running_mem_std:.2f} MB\n\n")
            
            f.write("="*50 + "\n")
            f.write("算法消耗（增量）\n")
            f.write("="*50 + "\n\n")
            
            f.write(f"  CPU 增量:    {cpu_delta:+.2f} % "
                   f"({cpu_delta / self.baseline_cpu * 100:+.1f}% 相对基线)\n")
            f.write(f"  内存增量:    {memory_delta:+.2f} MB "
                   f"({memory_delta / self.baseline_memory * 100:+.1f}% 相对基线)\n\n")
            
            f.write("="*50 + "\n")
            f.write("说明:\n")
            f.write("- 基线: 算法启动前的系统资源占用\n")
            f.write("- 运行时: 算法运行时的系统资源占用\n")
            f.write("- 增量: 算法实际消耗 = 运行时 - 基线\n")
            f.write("="*50 + "\n")
        
        rospy.loginfo(f"✓ 保存文本报告: {txt_path}")
    
    def print_summary(self):
        """打印评估总结"""
        cpu_delta = self.running_cpu - self.baseline_cpu
        memory_delta = self.running_memory - self.baseline_memory
        
        rospy.loginfo("="*50)
        rospy.loginfo(f"{self.algorithm_name} 性能评估结果")
        rospy.loginfo("="*50)
        
        rospy.loginfo("系统基线（算法启动前）:")
        rospy.loginfo(f"  CPU:    {self.baseline_cpu:.2f}%")
        rospy.loginfo(f"  内存:   {self.baseline_memory:.2f} MB")
        
        rospy.loginfo("算法运行时:")
        rospy.loginfo(f"  CPU:    {self.running_cpu:.2f}%")
        rospy.loginfo(f"  内存:   {self.running_memory:.2f} MB")
        
        rospy.loginfo("算法消耗（增量）:")
        rospy.loginfo(f"  CPU:    {cpu_delta:+.2f}% "
                     f"({cpu_delta / self.baseline_cpu * 100:+.1f}% 相对基线)")
        rospy.loginfo(f"  内存:   {memory_delta:+.2f} MB "
                     f"({memory_delta / self.baseline_memory * 100:+.1f}% 相对基线)")
        
        rospy.loginfo(f"结果已保存到: {self.output_dir}")
        rospy.loginfo("="*50)


def main():
    try:
        evaluator = SingleAlgorithmPerformanceEvaluator()
        
        rospy.loginfo("开始性能评估...")
        rospy.loginfo("按照提示操作即可")
        
        evaluator.run_evaluation()
        
        rospy.loginfo("性能评估完成！")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("评估被中断")
    except KeyboardInterrupt:
        rospy.loginfo("评估被用户停止")
    except Exception as e:
        rospy.logerr(f"评估过程中出错: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
