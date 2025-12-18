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
        
        # 算法颜色映射 - 高对比度颜色
        self.algo_colors = {
            'M-detector': '#FF0000',  # 纯红色
            'FAPP': '#0080FF',        # 亮蓝色
            'LV-DOT': '#FFD700',      # 金黄色
            'LDOT': '#8000FF'         # 深紫色
        }
        
        self.algo_markers = {
            'M-detector': 'o',
            'FAPP': 's',
            'LV-DOT': '^',
            'LDOT': 'd'
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
    
    def _moving_average(self, values, window_size=5):
        """
        计算滑动平均
        Args:
            values: 原始数据
            window_size: 窗口大小
        Returns:
            平滑后的数据
        """
        if len(values) < window_size:
            return values
        
        # 使用numpy的卷积计算滑动平均
        weights = np.ones(window_size) / window_size
        smoothed = np.convolve(values, weights, mode='valid')
        
        # 为了保持长度一致，在开头填充原始值
        padding = values[:window_size-1]
        return np.concatenate([padding, smoothed])
    
    def plot_time_series(self, algo_metrics_dict, save_path=None, timestamp=None):
        """
        绘制时序对比曲线图 - 2x2网格布局 + 滑动平均线
        Args:
            algo_metrics_dict: {
                'M-detector': {'timestamps': [...], 'metrics': [...]},
                'FAPP': {...},
                ...
            }
            save_path: 保存路径,默认为output_dir/time_series.png
            timestamp: 时间戳字符串，用于文件名
        """
        if save_path is None:
            filename = f'time_series_{timestamp}.png' if timestamp else 'time_series.png'
            save_path = os.path.join(self.output_dir, filename)
        
        # 创建2x2网格布局
        fig, axes = plt.subplots(2, 2, figsize=(16, 10))
        fig.suptitle('Multi-Algorithm Performance Over Time', fontsize=18, fontweight='bold', y=0.995)
        
        metric_names = ['recall', 'precision', 'f1', 'motp']
        metric_titles = ['Recall', 'Precision', 'F1-Score', 'MOTP (Localization Error)']
        y_labels = ['Recall', 'Precision', 'F1-Score', 'MOTP (m)']
        
        # 2x2布局的位置映射
        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        
        for idx, (metric, title, ylabel) in enumerate(zip(metric_names, metric_titles, y_labels)):
            row, col = positions[idx]
            ax = axes[row, col]
            
            for algo_name, data in algo_metrics_dict.items():
                if len(data['timestamps']) == 0:
                    continue
                
                timestamps = np.array(data['timestamps'])
                # 时间归零
                if len(timestamps) > 0:
                    timestamps = timestamps - timestamps[0]
                
                values = np.array([m[metric] for m in data['metrics']])
                
                # 计算滑动平均（窗口大小根据数据点数量自适应）
                window_size = max(3, min(7, len(values) // 10))
                smoothed_values = self._moving_average(values, window_size)
                
                color = self.algo_colors.get(algo_name, '#95A5A6')
                marker = self.algo_markers.get(algo_name, 'x')
                
                # 绘制原始数据点（半透明，小标记）
                ax.plot(timestamps, values, 
                       color=color,
                       marker=marker,
                       markevery=max(1, len(timestamps)//20),
                       markersize=4,
                       linewidth=1,
                       alpha=0.3,
                       linestyle=':')
                
                # 绘制滑动平均线（实线，粗线条）
                ax.plot(timestamps, smoothed_values, 
                       label=algo_name,
                       color=color,
                       linewidth=2.5,
                       alpha=0.95)
            
            # 设置标签和标题
            ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
            ax.set_ylabel(ylabel, fontsize=12, fontweight='bold')
            ax.set_title(title, fontsize=13, fontweight='bold', pad=8)
            
            # 图例 - 只在右上角子图显示
            if idx == 1:
                ax.legend(loc='upper right', fontsize=10, framealpha=0.95, 
                         ncol=2, borderaxespad=0.5)
            
            # 网格样式
            ax.grid(True, alpha=0.3, linestyle='--', linewidth=0.7)
            ax.set_axisbelow(True)
            
            # Y轴范围优化
            if metric == 'motp':
                ax.set_ylim(bottom=0)
            else:
                ax.set_ylim(0, 1.05)  # Recall/Precision/F1 范围 0-1
            
            # 添加参考线
            if metric != 'motp':
                ax.axhline(y=0.8, color='gray', linestyle=':', linewidth=1.2, alpha=0.6)
                ax.text(0.02, 0.82, '0.8', transform=ax.get_yaxis_transform(),
                       fontsize=9, color='gray', alpha=0.8, fontweight='bold')
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.png', '.pdf'), dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Time series plot saved to: {save_path}")
    
    def plot_radar_chart(self, algo_overall_metrics, save_path=None, timestamp=None):
        """
        绘制雷达图
        Args:
            algo_overall_metrics: {
                'M-detector': {'recall_mean': 0.9, 'precision_mean': 0.85, ...},
                'FAPP': {...},
                ...
            }
            save_path: 保存路径
            timestamp: 时间戳字符串，用于文件名
        """
        if save_path is None:
            filename = f'radar_chart_{timestamp}.png' if timestamp else 'radar_chart.png'
            save_path = os.path.join(self.output_dir, filename)
        
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
                 size=16, fontweight='bold', pad=5)
        plt.legend(loc='upper right', bbox_to_anchor=(1.3, 1.1))
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.savefig(save_path.replace('.png', '.pdf'), dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f"Radar chart saved to: {save_path}")
    
    def generate_performance_table(self, algo_overall_metrics, save_path=None, timestamp=None):
        """
        生成性能汇总表格
        Args:
            algo_overall_metrics: {
                'M-detector': {'recall_mean': 0.9, 'recall_std': 0.05, ...},
                'FAPP': {...},
                ...
            }
            save_path: 保存路径
            timestamp: 时间戳字符串，用于文件名
        """
        if save_path is None:
            filename = f'performance_table_{timestamp}.txt' if timestamp else 'performance_table.txt'
            save_path = os.path.join(self.output_dir, filename)
        
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
                 fontsize=16, fontweight='bold', pad=5)
        
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
