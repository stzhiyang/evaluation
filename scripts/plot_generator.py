#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plot Generator Module
生成对比图表:时序曲线图、雷达图、性能表格
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt
from matplotlib import font_manager
import os


class PlotGenerator:
    """图表生成器"""
    
    def __init__(self, output_dir='results'):
        """
        Args:
            output_dir: 输出目录
        """
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # 配置matplotlib
        self._setup_matplotlib()
        
        # 算法颜色映射
        self.algo_colors = {
            'M-detector': '#E74C3C',  # 红色
            'FAPP': '#3498DB',        # 蓝色
            'LV-DOT': '#2ECC71',      # 绿色
            'Algorithm4': '#F39C12'   # 橙色
        }
        
        self.algo_markers = {
            'M-detector': 'o',
            'FAPP': 's',
            'LV-DOT': '^',
            'Algorithm4': 'd'
        }
    
    def _setup_matplotlib(self):
        """配置matplotlib样式"""
        plt.style.use('seaborn-v0_8-darkgrid')
        
        # 使用标准字体
        plt.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Helvetica', 'sans-serif']
        plt.rcParams['figure.figsize'] = (12, 8)
        plt.rcParams['axes.labelsize'] = 12
        plt.rcParams['axes.titlesize'] = 14
        plt.rcParams['xtick.labelsize'] = 10
        plt.rcParams['ytick.labelsize'] = 10
        plt.rcParams['legend.fontsize'] = 10
        plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    
    def plot_time_series(self, algo_metrics_dict, save_path=None):
        """
        绘制时序对比曲线图
        Args:
            algo_metrics_dict: {
                'M-detector': {'timestamps': [...], 'metrics': [...]},
                'FAPP': {...},
                ...
            }
            save_path: 保存路径,默认为output_dir/time_series.png
        """
        if save_path is None:
            save_path = os.path.join(self.output_dir, 'time_series.png')
        
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Multi-Algorithm Performance Comparison Over Time', fontsize=16, fontweight='bold')
        
        metric_names = ['recall', 'precision', 'f1', 'motp']
        metric_titles = ['Recall', 'Precision', 'F1-Score', 'MOTP (m)']
        
        for idx, (metric, title) in enumerate(zip(metric_names, metric_titles)):
            ax = axes[idx // 2, idx % 2]
            
            for algo_name, data in algo_metrics_dict.items():
                if len(data['timestamps']) == 0:
                    continue
                
                timestamps = np.array(data['timestamps'])
                # 时间归零
                if len(timestamps) > 0:
                    timestamps = timestamps - timestamps[0]
                
                values = [m[metric] for m in data['metrics']]
                
                # 绘制曲线
                ax.plot(timestamps, values, 
                       label=algo_name,
                       color=self.algo_colors.get(algo_name, '#95A5A6'),
                       marker=self.algo_markers.get(algo_name, 'x'),
                       markevery=max(1, len(timestamps)//20),
                       linewidth=2,
                       alpha=0.8)
            
            ax.set_xlabel('Time (s)', fontsize=11)
            ax.set_ylabel(title, fontsize=11)
            ax.set_title(f'{title} vs Time', fontsize=12, fontweight='bold')
            ax.legend(loc='best')
            ax.grid(True, alpha=0.3)
            
            # 为MOTP设置合适的y轴范围
            if metric == 'motp':
                ax.set_ylim(bottom=0)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.png', '.pdf'), dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Time series plot saved to: {save_path}")
    
    def plot_radar_chart(self, algo_overall_metrics, save_path=None):
        """
        绘制雷达图
        Args:
            algo_overall_metrics: {
                'M-detector': {'recall_mean': 0.9, 'precision_mean': 0.85, ...},
                'FAPP': {...},
                ...
            }
            save_path: 保存路径
        """
        if save_path is None:
            save_path = os.path.join(self.output_dir, 'radar_chart.png')
        
        # 准备数据
        categories = ['Recall', 'Precision', 'F1-Score', 'MOTP\n(normalized)']
        num_vars = len(categories)
        
        # 计算角度
        angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
        angles += angles[:1]  # 闭合
        
        fig, ax = plt.subplots(figsize=(10, 10), subplot_kw=dict(projection='polar'))
        
        for algo_name, metrics in algo_overall_metrics.items():
            # 提取指标值
            recall = metrics.get('recall_mean', 0)
            precision = metrics.get('precision_mean', 0)
            f1 = metrics.get('f1_mean', 0)
            motp = metrics.get('motp_mean', 0)
            
            # MOTP归一化(反转并缩放到0-1,越小越好)
            # 假设MOTP在0-2m范围内
            motp_normalized = max(0, 1 - motp / 2.0)
            
            values = [recall, precision, f1, motp_normalized]
            values += values[:1]  # 闭合
            
            # 绘制
            ax.plot(angles, values, 
                   'o-', 
                   linewidth=2,
                   label=algo_name,
                   color=self.algo_colors.get(algo_name, '#95A5A6'))
            ax.fill(angles, values, 
                   alpha=0.15,
                   color=self.algo_colors.get(algo_name, '#95A5A6'))
        
        # 设置标签
        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, fontsize=12)
        ax.set_ylim(0, 1)
        ax.set_yticks([0.2, 0.4, 0.6, 0.8, 1.0])
        ax.set_yticklabels(['0.2', '0.4', '0.6', '0.8', '1.0'], fontsize=10)
        ax.grid(True, linestyle='--', alpha=0.5)
        
        plt.title('Multi-Algorithm Performance Radar Chart', 
                 size=16, fontweight='bold', pad=20)
        plt.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.png', '.pdf'), dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Radar chart saved to: {save_path}")
    
    def generate_performance_table(self, algo_overall_metrics, save_path=None):
        """
        生成性能汇总表格
        Args:
            algo_overall_metrics: {
                'M-detector': {'recall_mean': 0.9, 'recall_std': 0.05, ...},
                'FAPP': {...},
                ...
            }
            save_path: 保存路径
        """
        if save_path is None:
            save_path = os.path.join(self.output_dir, 'performance_table.txt')
        
        # 创建表格
        fig, ax = plt.subplots(figsize=(12, len(algo_overall_metrics) * 1.2 + 1))
        ax.axis('tight')
        ax.axis('off')
        
        # 表头
        columns = ['Algorithm', 'Recall ↑', 'Precision ↑', 'F1-Score ↑', 'MOTP (m) ↓']
        
        # 数据
        table_data = []
        for algo_name, metrics in algo_overall_metrics.items():
            recall_str = f"{metrics.get('recall_mean', 0):.3f} ± {metrics.get('recall_std', 0):.3f}"
            precision_str = f"{metrics.get('precision_mean', 0):.3f} ± {metrics.get('precision_std', 0):.3f}"
            f1_str = f"{metrics.get('f1_mean', 0):.3f} ± {metrics.get('f1_std', 0):.3f}"
            motp_str = f"{metrics.get('motp_mean', 0):.3f} ± {metrics.get('motp_std', 0):.3f}"
            
            table_data.append([algo_name, recall_str, precision_str, f1_str, motp_str])
        
        # 创建表格
        table = ax.table(cellText=table_data,
                        colLabels=columns,
                        cellLoc='center',
                        loc='center',
                        colWidths=[0.2, 0.2, 0.2, 0.2, 0.2])
        
        table.auto_set_font_size(False)
        table.set_fontsize(11)
        table.scale(1, 2)
        
        # 设置表头样式
        for i in range(len(columns)):
            table[(0, i)].set_facecolor('#3498DB')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # 设置行颜色
        for i, algo_name in enumerate(algo_overall_metrics.keys()):
            color = self.algo_colors.get(algo_name, '#ECF0F1')
            for j in range(len(columns)):
                table[(i+1, j)].set_facecolor(color)
                table[(i+1, j)].set_alpha(0.3)
        
        plt.title('Multi-Algorithm Performance Summary', 
                 fontsize=16, fontweight='bold', pad=20)
        
        plt.savefig(save_path.replace('.txt', '.png'), dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.txt', '.pdf'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 同时保存文本格式
        with open(save_path, 'w') as f:
            f.write("=" * 100 + "\n")
            f.write("Multi-Algorithm Performance Summary\n")
            f.write("=" * 100 + "\n\n")
            
            # 写入表格
            header = f"{'Algorithm':<20} {'Recall ↑':<25} {'Precision ↑':<25} {'F1-Score ↑':<25} {'MOTP (m) ↓':<25}\n"
            f.write(header)
            f.write("-" * 100 + "\n")
            
            for algo_name, metrics in algo_overall_metrics.items():
                recall_str = f"{metrics.get('recall_mean', 0):.3f} ± {metrics.get('recall_std', 0):.3f}"
                precision_str = f"{metrics.get('precision_mean', 0):.3f} ± {metrics.get('precision_std', 0):.3f}"
                f1_str = f"{metrics.get('f1_mean', 0):.3f} ± {metrics.get('f1_std', 0):.3f}"
                motp_str = f"{metrics.get('motp_mean', 0):.3f} ± {metrics.get('motp_std', 0):.3f}"
                
                row = f"{algo_name:<20} {recall_str:<25} {precision_str:<25} {f1_str:<25} {motp_str:<25}\n"
                f.write(row)
            
            f.write("=" * 100 + "\n")
        
        print(f"Performance table saved to: {save_path}")


if __name__ == '__main__':
    # 测试代码
    generator = PlotGenerator()
    print("PlotGenerator initialized successfully")
