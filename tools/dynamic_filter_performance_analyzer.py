#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动态过滤器性能分析工具
深入分析动态对象过滤器的性能瓶颈、资源使用和优化机会

作者: Claude Code
创建时间: 2025-01-08
功能:
1. 算法复杂度分析
2. CPU和内存性能监控
3. 实时性能测量
4. 瓶颈识别和优化建议
5. 多线程性能分析
"""

import psutil
import time
import threading
import queue
import statistics
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import json
import logging
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, asdict
from datetime import datetime
import subprocess
import os
import re
import pickle

# 设置matplotlib中文字体支持
plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

@dataclass
class PerformanceMetrics:
    """性能指标数据结构"""
    timestamp: float
    cpu_usage: float
    memory_usage: float  # MB
    memory_rss: float   # MB
    memory_vms: float   # MB
    thread_count: int
    processing_time: float  # ms
    points_processed: int
    dynamic_points_detected: int
    filter_efficiency: float
    throughput: float  # points/sec

    def to_dict(self) -> Dict:
        return asdict(self)

@dataclass
class AlgorithmComplexity:
    """算法复杂度分析结果"""
    temporal_analysis_complexity: str
    geometric_verification_complexity: str
    kdtree_operations: int
    memory_allocations: int
    parallel_efficiency: float
    bottleneck_functions: List[str]

@dataclass
class PerformanceReport:
    """性能分析报告"""
    analysis_duration: float
    avg_cpu_usage: float
    peak_memory_usage: float
    avg_processing_time: float
    throughput_stats: Dict[str, float]
    bottlenecks: List[str]
    optimization_recommendations: List[str]
    complexity_analysis: AlgorithmComplexity
    resource_efficiency: Dict[str, float]

class DynamicFilterProfiler:
    """动态过滤器性能分析器"""

    def __init__(self, log_level: str = "INFO"):
        self.logger = self._setup_logger(log_level)
        self.metrics_queue = queue.Queue()
        self.monitoring_active = False
        self.process_pid = None
        self.metrics_history: List[PerformanceMetrics] = []
        self.analysis_start_time = None

        # 性能阈值配置
        self.performance_thresholds = {
            'max_cpu_usage': 80.0,           # %
            'max_memory_usage': 1024.0,      # MB
            'max_processing_time': 50.0,     # ms
            'min_throughput': 10000.0,       # points/sec
            'max_memory_growth': 100.0,      # MB/min
        }

    def _setup_logger(self, level: str) -> logging.Logger:
        """设置日志系统"""
        logger = logging.getLogger('DynamicFilterProfiler')
        logger.setLevel(getattr(logging, level.upper()))

        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter(
                '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
            )
            handler.setFormatter(formatter)
            logger.addHandler(handler)

        return logger

    def attach_to_process(self, process_name: str = "localizer_node") -> bool:
        """附加到目标进程进行监控"""
        try:
            # 查找进程
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if process_name in proc.info['name'] or \
                       any(process_name in cmd for cmd in proc.info['cmdline'] if cmd):
                        self.process_pid = proc.info['pid']
                        self.logger.info(f"已附加到进程: {process_name} (PID: {self.process_pid})")
                        return True
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue

            self.logger.warning(f"未找到进程: {process_name}")
            return False

        except Exception as e:
            self.logger.error(f"附加进程失败: {e}")
            return False

    def start_monitoring(self, duration: float = 60.0) -> None:
        """开始性能监控"""
        if not self.process_pid:
            self.logger.error("未指定监控进程")
            return

        self.monitoring_active = True
        self.analysis_start_time = time.time()
        self.metrics_history.clear()

        self.logger.info(f"开始监控进程 {self.process_pid}，持续时间: {duration}秒")

        # 启动监控线程
        monitor_thread = threading.Thread(
            target=self._monitor_process,
            args=(duration,),
            daemon=True
        )
        monitor_thread.start()

        # 等待监控完成
        monitor_thread.join()

        self.monitoring_active = False
        self.logger.info("监控完成")

    def _monitor_process(self, duration: float) -> None:
        """进程监控线程"""
        try:
            process = psutil.Process(self.process_pid)
            start_time = time.time()

            while self.monitoring_active and (time.time() - start_time) < duration:
                try:
                    # 获取进程信息
                    cpu_percent = process.cpu_percent()
                    memory_info = process.memory_info()
                    thread_count = process.num_threads()

                    # 创建性能指标
                    metrics = PerformanceMetrics(
                        timestamp=time.time(),
                        cpu_usage=cpu_percent,
                        memory_usage=memory_info.rss / (1024 * 1024),  # MB
                        memory_rss=memory_info.rss / (1024 * 1024),
                        memory_vms=memory_info.vms / (1024 * 1024),
                        thread_count=thread_count,
                        processing_time=0.0,  # 需要从应用程序获取
                        points_processed=0,    # 需要从应用程序获取
                        dynamic_points_detected=0,  # 需要从应用程序获取
                        filter_efficiency=0.0,
                        throughput=0.0
                    )

                    self.metrics_history.append(metrics)

                    # 检查性能阈值
                    self._check_performance_thresholds(metrics)

                    time.sleep(0.1)  # 100ms采样间隔

                except psutil.NoSuchProcess:
                    self.logger.warning("目标进程已终止")
                    break
                except Exception as e:
                    self.logger.error(f"监控过程中出错: {e}")
                    time.sleep(0.1)

        except Exception as e:
            self.logger.error(f"监控线程异常: {e}")

    def _check_performance_thresholds(self, metrics: PerformanceMetrics) -> None:
        """检查性能阈值并发出警告"""
        if metrics.cpu_usage > self.performance_thresholds['max_cpu_usage']:
            self.logger.warning(f"CPU使用率过高: {metrics.cpu_usage:.1f}%")

        if metrics.memory_usage > self.performance_thresholds['max_memory_usage']:
            self.logger.warning(f"内存使用量过高: {metrics.memory_usage:.1f}MB")

    def analyze_algorithm_complexity(self) -> AlgorithmComplexity:
        """分析算法复杂度"""
        self.logger.info("分析算法复杂度...")

        # 基于代码分析的复杂度评估
        bottleneck_functions = [
            "computeTemporalInfo - O(N*H*K)",
            "geometricConsistencyCheck - O(N*K)",
            "computeNormals - O(N*K)",
            "findCorrespondingPoints - O(N*log(M))",
            "KD树构建 - O(N*log(N))"
        ]

        # 估算操作数量
        kdtree_operations = 0
        memory_allocations = 0

        if self.metrics_history:
            avg_points = statistics.mean([m.points_processed for m in self.metrics_history if m.points_processed > 0]) or 10000
            history_frames = 5  # 默认历史帧数
            neighbors = 10      # 默认邻居数

            # 每次处理的KD树操作估算
            kdtree_operations = int(avg_points * history_frames + avg_points * neighbors)

            # 内存分配估算
            memory_allocations = int(avg_points * 3 + history_frames * avg_points)

        # 并行效率分析
        parallel_efficiency = self._estimate_parallel_efficiency()

        return AlgorithmComplexity(
            temporal_analysis_complexity="O(N*H*K) - N:点数, H:历史帧数, K:邻居数",
            geometric_verification_complexity="O(N*K) - N:点数, K:邻居数",
            kdtree_operations=kdtree_operations,
            memory_allocations=memory_allocations,
            parallel_efficiency=parallel_efficiency,
            bottleneck_functions=bottleneck_functions
        )

    def _estimate_parallel_efficiency(self) -> float:
        """估算并行处理效率"""
        if not self.metrics_history:
            return 0.0

        # 基于CPU使用率和线程数估算并行效率
        cpu_usages = [m.cpu_usage for m in self.metrics_history]
        thread_counts = [m.thread_count for m in self.metrics_history]

        if not cpu_usages or not thread_counts:
            return 0.0

        avg_cpu = statistics.mean(cpu_usages)
        avg_threads = statistics.mean(thread_counts)

        # 理想情况下，并行效率 = 实际CPU使用率 / (线程数 * 100%)
        if avg_threads > 1:
            efficiency = min(avg_cpu / (avg_threads * 25), 1.0)  # 假设有效线程为总线程的1/4
        else:
            efficiency = avg_cpu / 100.0

        return efficiency

    def analyze_memory_patterns(self) -> Dict[str, Any]:
        """分析内存使用模式"""
        if not self.metrics_history:
            return {}

        memory_usages = [m.memory_usage for m in self.metrics_history]
        timestamps = [m.timestamp for m in self.metrics_history]

        # 计算内存统计
        memory_stats = {
            'peak_usage': max(memory_usages),
            'average_usage': statistics.mean(memory_usages),
            'min_usage': min(memory_usages),
            'std_deviation': statistics.stdev(memory_usages) if len(memory_usages) > 1 else 0,
            'growth_rate': 0.0,
            'fragmentation_indicator': 0.0
        }

        # 计算内存增长率
        if len(timestamps) > 1:
            duration_minutes = (timestamps[-1] - timestamps[0]) / 60.0
            memory_growth = memory_usages[-1] - memory_usages[0]
            memory_stats['growth_rate'] = memory_growth / duration_minutes if duration_minutes > 0 else 0

        # 内存碎片指标（基于RSS和VMS的差异）
        if self.metrics_history:
            rss_values = [m.memory_rss for m in self.metrics_history]
            vms_values = [m.memory_vms for m in self.metrics_history]
            if rss_values and vms_values:
                avg_rss = statistics.mean(rss_values)
                avg_vms = statistics.mean(vms_values)
                memory_stats['fragmentation_indicator'] = (avg_vms - avg_rss) / avg_vms if avg_vms > 0 else 0

        return memory_stats

    def identify_performance_bottlenecks(self) -> List[str]:
        """识别性能瓶颈"""
        bottlenecks = []

        if not self.metrics_history:
            return ["无性能数据可供分析"]

        # CPU使用率分析
        cpu_usages = [m.cpu_usage for m in self.metrics_history]
        avg_cpu = statistics.mean(cpu_usages)
        max_cpu = max(cpu_usages)

        if max_cpu > 90:
            bottlenecks.append(f"CPU使用率过高 (峰值: {max_cpu:.1f}%)")
        if avg_cpu > 70:
            bottlenecks.append(f"平均CPU使用率较高 ({avg_cpu:.1f}%)")

        # 内存使用分析
        memory_stats = self.analyze_memory_patterns()
        if memory_stats.get('growth_rate', 0) > 50:  # 50MB/min
            bottlenecks.append(f"内存增长过快 ({memory_stats['growth_rate']:.1f} MB/min)")

        if memory_stats.get('peak_usage', 0) > 1024:  # 1GB
            bottlenecks.append(f"峰值内存使用量过高 ({memory_stats['peak_usage']:.1f} MB)")

        # 处理时间分析
        processing_times = [m.processing_time for m in self.metrics_history if m.processing_time > 0]
        if processing_times:
            avg_time = statistics.mean(processing_times)
            max_time = max(processing_times)

            if max_time > 100:  # 100ms
                bottlenecks.append(f"处理时间过长 (峰值: {max_time:.1f}ms)")
            if avg_time > 50:   # 50ms
                bottlenecks.append(f"平均处理时间较长 ({avg_time:.1f}ms)")

        # 并行效率分析
        parallel_efficiency = self._estimate_parallel_efficiency()
        if parallel_efficiency < 0.5:
            bottlenecks.append(f"并行处理效率低 ({parallel_efficiency:.1%})")

        return bottlenecks if bottlenecks else ["未发现明显性能瓶颈"]

    def generate_optimization_recommendations(self) -> List[str]:
        """生成优化建议"""
        recommendations = []

        # 基于性能瓶颈生成建议
        bottlenecks = self.identify_performance_bottlenecks()

        for bottleneck in bottlenecks:
            if "CPU使用率" in bottleneck:
                recommendations.extend([
                    "优化热点函数，减少不必要的计算",
                    "使用更高效的数据结构（如空间索引）",
                    "实现渐进式处理，避免一次性处理大量数据",
                    "优化循环和算法逻辑"
                ])

            if "内存" in bottleneck:
                recommendations.extend([
                    "实现智能内存管理和对象池化",
                    "优化历史数据存储策略",
                    "使用内存映射文件减少内存占用",
                    "实现自适应降采样策略"
                ])

            if "处理时间" in bottleneck:
                recommendations.extend([
                    "优化KD树构建和查询算法",
                    "实现多级缓存策略",
                    "使用SIMD指令加速数值计算",
                    "优化PCL库的使用方式"
                ])

            if "并行" in bottleneck:
                recommendations.extend([
                    "重新设计并行算法，减少线程间竞争",
                    "使用无锁数据结构",
                    "优化任务分割策略",
                    "实现工作窃取算法"
                ])

        # 通用优化建议
        general_recommendations = [
            "实现预测性降采样，根据场景动态调整点云密度",
            "使用GPU加速关键计算（CUDA PCL）",
            "实现增量式历史数据更新",
            "优化体素滤波器参数",
            "实现自适应参数调优",
            "添加性能监控和自动调优机制"
        ]

        recommendations.extend(general_recommendations)

        # 去重
        return list(set(recommendations))

    def generate_performance_report(self) -> PerformanceReport:
        """生成综合性能报告"""
        if not self.metrics_history:
            raise ValueError("无性能数据，无法生成报告")

        # 计算分析持续时间
        analysis_duration = (self.metrics_history[-1].timestamp -
                           self.metrics_history[0].timestamp) if len(self.metrics_history) > 1 else 0

        # CPU统计
        cpu_usages = [m.cpu_usage for m in self.metrics_history]
        avg_cpu_usage = statistics.mean(cpu_usages)

        # 内存统计
        memory_stats = self.analyze_memory_patterns()
        peak_memory_usage = memory_stats.get('peak_usage', 0)

        # 处理时间统计
        processing_times = [m.processing_time for m in self.metrics_history if m.processing_time > 0]
        avg_processing_time = statistics.mean(processing_times) if processing_times else 0

        # 吞吐量统计
        throughputs = [m.throughput for m in self.metrics_history if m.throughput > 0]
        throughput_stats = {
            'average': statistics.mean(throughputs) if throughputs else 0,
            'peak': max(throughputs) if throughputs else 0,
            'min': min(throughputs) if throughputs else 0,
            'std_dev': statistics.stdev(throughputs) if len(throughputs) > 1 else 0
        }

        # 资源效率
        resource_efficiency = {
            'cpu_efficiency': avg_cpu_usage / 100.0,
            'memory_efficiency': memory_stats.get('average_usage', 0) / memory_stats.get('peak_usage', 1),
            'parallel_efficiency': self._estimate_parallel_efficiency(),
            'overall_efficiency': 0.0
        }

        # 计算整体效率
        resource_efficiency['overall_efficiency'] = (
            resource_efficiency['cpu_efficiency'] * 0.4 +
            resource_efficiency['memory_efficiency'] * 0.3 +
            resource_efficiency['parallel_efficiency'] * 0.3
        )

        return PerformanceReport(
            analysis_duration=analysis_duration,
            avg_cpu_usage=avg_cpu_usage,
            peak_memory_usage=peak_memory_usage,
            avg_processing_time=avg_processing_time,
            throughput_stats=throughput_stats,
            bottlenecks=self.identify_performance_bottlenecks(),
            optimization_recommendations=self.generate_optimization_recommendations(),
            complexity_analysis=self.analyze_algorithm_complexity(),
            resource_efficiency=resource_efficiency
        )

    def save_metrics_data(self, filepath: str) -> None:
        """保存性能数据"""
        try:
            data = {
                'metrics_history': [m.to_dict() for m in self.metrics_history],
                'analysis_timestamp': datetime.now().isoformat(),
                'process_pid': self.process_pid
            }

            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

            self.logger.info(f"性能数据已保存到: {filepath}")

        except Exception as e:
            self.logger.error(f"保存性能数据失败: {e}")

    def load_metrics_data(self, filepath: str) -> bool:
        """加载性能数据"""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)

            self.metrics_history = [
                PerformanceMetrics(**m) for m in data['metrics_history']
            ]

            self.logger.info(f"已加载 {len(self.metrics_history)} 条性能记录")
            return True

        except Exception as e:
            self.logger.error(f"加载性能数据失败: {e}")
            return False

class PerformanceVisualizer:
    """性能可视化器"""

    def __init__(self):
        self.fig_size = (15, 10)

    def create_performance_dashboard(self, profiler: DynamicFilterProfiler,
                                   save_path: Optional[str] = None) -> None:
        """创建性能监控仪表板"""
        if not profiler.metrics_history:
            print("无性能数据可供可视化")
            return

        # 创建子图
        fig, axes = plt.subplots(2, 3, figsize=self.fig_size)
        fig.suptitle('动态过滤器性能监控仪表板', fontsize=16, fontweight='bold')

        # 提取数据
        timestamps = [m.timestamp for m in profiler.metrics_history]
        start_time = timestamps[0] if timestamps else 0
        relative_times = [(t - start_time) for t in timestamps]

        cpu_usages = [m.cpu_usage for m in profiler.metrics_history]
        memory_usages = [m.memory_usage for m in profiler.metrics_history]
        thread_counts = [m.thread_count for m in profiler.metrics_history]
        processing_times = [m.processing_time for m in profiler.metrics_history if m.processing_time > 0]
        throughputs = [m.throughput for m in profiler.metrics_history if m.throughput > 0]

        # 1. CPU使用率趋势
        axes[0, 0].plot(relative_times, cpu_usages, 'b-', linewidth=2)
        axes[0, 0].axhline(y=80, color='r', linestyle='--', alpha=0.7, label='警告阈值')
        axes[0, 0].set_title('CPU使用率 (%)')
        axes[0, 0].set_xlabel('时间 (秒)')
        axes[0, 0].set_ylabel('CPU使用率 (%)')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].legend()

        # 2. 内存使用量趋势
        axes[0, 1].plot(relative_times, memory_usages, 'g-', linewidth=2)
        axes[0, 1].axhline(y=1024, color='r', linestyle='--', alpha=0.7, label='警告阈值')
        axes[0, 1].set_title('内存使用量 (MB)')
        axes[0, 1].set_xlabel('时间 (秒)')
        axes[0, 1].set_ylabel('内存使用量 (MB)')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].legend()

        # 3. 线程数变化
        axes[0, 2].plot(relative_times, thread_counts, 'orange', linewidth=2)
        axes[0, 2].set_title('线程数量')
        axes[0, 2].set_xlabel('时间 (秒)')
        axes[0, 2].set_ylabel('线程数')
        axes[0, 2].grid(True, alpha=0.3)

        # 4. 处理时间分布（如果有数据）
        if processing_times:
            axes[1, 0].hist(processing_times, bins=20, alpha=0.7, color='purple', edgecolor='black')
            axes[1, 0].axvline(x=50, color='r', linestyle='--', alpha=0.7, label='目标阈值')
            axes[1, 0].set_title('处理时间分布 (ms)')
            axes[1, 0].set_xlabel('处理时间 (ms)')
            axes[1, 0].set_ylabel('频次')
            axes[1, 0].legend()
        else:
            axes[1, 0].text(0.5, 0.5, '无处理时间数据', ha='center', va='center', transform=axes[1, 0].transAxes)
            axes[1, 0].set_title('处理时间分布')

        # 5. 吞吐量趋势（如果有数据）
        if throughputs and len(throughputs) > 1:
            throughput_times = relative_times[:len(throughputs)]
            axes[1, 1].plot(throughput_times, throughputs, 'red', linewidth=2)
            axes[1, 1].set_title('处理吞吐量 (points/sec)')
            axes[1, 1].set_xlabel('时间 (秒)')
            axes[1, 1].set_ylabel('吞吐量')
            axes[1, 1].grid(True, alpha=0.3)
        else:
            axes[1, 1].text(0.5, 0.5, '无吞吐量数据', ha='center', va='center', transform=axes[1, 1].transAxes)
            axes[1, 1].set_title('处理吞吐量')

        # 6. 资源使用效率图
        if cpu_usages and memory_usages:
            scatter = axes[1, 2].scatter(cpu_usages, memory_usages,
                                       c=relative_times, cmap='viridis', alpha=0.6)
            axes[1, 2].set_title('CPU vs 内存使用率')
            axes[1, 2].set_xlabel('CPU使用率 (%)')
            axes[1, 2].set_ylabel('内存使用量 (MB)')
            axes[1, 2].grid(True, alpha=0.3)

            # 添加颜色条
            cbar = plt.colorbar(scatter, ax=axes[1, 2])
            cbar.set_label('时间 (秒)')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"性能仪表板已保存到: {save_path}")

        plt.show()

    def create_bottleneck_analysis_chart(self, report: PerformanceReport,
                                        save_path: Optional[str] = None) -> None:
        """创建瓶颈分析图表"""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=self.fig_size)
        fig.suptitle('性能瓶颈分析报告', fontsize=16, fontweight='bold')

        # 1. 资源效率雷达图
        categories = ['CPU效率', '内存效率', '并行效率', '整体效率']
        values = [
            report.resource_efficiency['cpu_efficiency'],
            report.resource_efficiency['memory_efficiency'],
            report.resource_efficiency['parallel_efficiency'],
            report.resource_efficiency['overall_efficiency']
        ]

        angles = np.linspace(0, 2*np.pi, len(categories), endpoint=False).tolist()
        values += values[:1]  # 闭合图形
        angles += angles[:1]

        ax1 = plt.subplot(2, 2, 1, projection='polar')
        ax1.plot(angles, values, 'o-', linewidth=2, color='blue')
        ax1.fill(angles, values, alpha=0.25, color='blue')
        ax1.set_xticks(angles[:-1])
        ax1.set_xticklabels(categories)
        ax1.set_ylim(0, 1)
        ax1.set_title('资源使用效率', y=1.08)

        # 2. 瓶颈严重程度
        bottleneck_labels = [b[:30] + '...' if len(b) > 30 else b for b in report.bottlenecks[:5]]
        bottleneck_scores = [1.0] * len(bottleneck_labels)  # 简化评分

        ax2 = plt.subplot(2, 2, 2)
        bars = ax2.barh(range(len(bottleneck_labels)), bottleneck_scores, color='red', alpha=0.7)
        ax2.set_yticks(range(len(bottleneck_labels)))
        ax2.set_yticklabels(bottleneck_labels)
        ax2.set_xlabel('严重程度')
        ax2.set_title('主要性能瓶颈')
        ax2.set_xlim(0, 1)

        # 3. 算法复杂度信息
        ax3 = plt.subplot(2, 2, 3)
        complexity_info = [
            f"时间复杂度: {report.complexity_analysis.temporal_analysis_complexity}",
            f"几何验证: {report.complexity_analysis.geometric_verification_complexity}",
            f"KD树操作: {report.complexity_analysis.kdtree_operations:,}",
            f"内存分配: {report.complexity_analysis.memory_allocations:,}",
            f"并行效率: {report.complexity_analysis.parallel_efficiency:.1%}"
        ]

        ax3.axis('off')
        for i, info in enumerate(complexity_info):
            ax3.text(0.05, 0.9 - i*0.15, info, transform=ax3.transAxes,
                    fontsize=10, verticalalignment='top')
        ax3.set_title('算法复杂度分析')

        # 4. 吞吐量统计
        ax4 = plt.subplot(2, 2, 4)
        throughput_labels = ['平均', '峰值', '最小']
        throughput_values = [
            report.throughput_stats['average'],
            report.throughput_stats['peak'],
            report.throughput_stats['min']
        ]

        bars = ax4.bar(throughput_labels, throughput_values, color=['blue', 'green', 'orange'])
        ax4.set_ylabel('吞吐量 (points/sec)')
        ax4.set_title('处理吞吐量统计')

        # 添加数值标签
        for bar, value in zip(bars, throughput_values):
            if value > 0:
                ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height(),
                        f'{value:.0f}', ha='center', va='bottom')

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"瓶颈分析图表已保存到: {save_path}")

        plt.show()

def main():
    """主函数"""
    print("=== 动态过滤器性能分析工具 ===")

    # 创建性能分析器
    profiler = DynamicFilterProfiler(log_level="INFO")

    # 尝试附加到进程
    if not profiler.attach_to_process("localizer_node"):
        print("警告: 未找到目标进程，将使用模拟数据进行演示")
        # 生成模拟数据用于演示
        generate_demo_data(profiler)
    else:
        # 开始监控
        duration = 30.0  # 监控30秒
        print(f"开始监控 {duration} 秒...")
        profiler.start_monitoring(duration)

    # 生成性能报告
    if profiler.metrics_history:
        print("\n生成性能报告...")
        report = profiler.generate_performance_report()

        # 打印报告摘要
        print_performance_summary(report)

        # 保存数据
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        data_file = f"/tmp/dynamic_filter_performance_{timestamp}.json"
        profiler.save_metrics_data(data_file)

        # 创建可视化
        visualizer = PerformanceVisualizer()

        dashboard_file = f"/tmp/performance_dashboard_{timestamp}.png"
        visualizer.create_performance_dashboard(profiler, dashboard_file)

        bottleneck_file = f"/tmp/bottleneck_analysis_{timestamp}.png"
        visualizer.create_bottleneck_analysis_chart(report, bottleneck_file)

        print(f"\n分析完成! 结果文件:")
        print(f"- 性能数据: {data_file}")
        print(f"- 性能仪表板: {dashboard_file}")
        print(f"- 瓶颈分析: {bottleneck_file}")
    else:
        print("无性能数据可供分析")

def generate_demo_data(profiler: DynamicFilterProfiler):
    """生成演示数据"""
    import random

    print("生成演示性能数据...")
    start_time = time.time()

    for i in range(100):
        # 模拟性能数据
        metrics = PerformanceMetrics(
            timestamp=start_time + i * 0.1,
            cpu_usage=random.uniform(30, 85),
            memory_usage=random.uniform(200, 800),
            memory_rss=random.uniform(180, 750),
            memory_vms=random.uniform(300, 1200),
            thread_count=random.randint(4, 12),
            processing_time=random.uniform(15, 65),
            points_processed=random.randint(8000, 15000),
            dynamic_points_detected=random.randint(100, 800),
            filter_efficiency=random.uniform(0.7, 0.95),
            throughput=random.uniform(8000, 20000)
        )
        profiler.metrics_history.append(metrics)

def print_performance_summary(report: PerformanceReport):
    """打印性能报告摘要"""
    print("\n" + "="*60)
    print("                    性能分析报告摘要")
    print("="*60)

    print(f"分析持续时间: {report.analysis_duration:.1f} 秒")
    print(f"平均CPU使用率: {report.avg_cpu_usage:.1f}%")
    print(f"峰值内存使用: {report.peak_memory_usage:.1f} MB")
    print(f"平均处理时间: {report.avg_processing_time:.1f} ms")

    print(f"\n吞吐量统计:")
    print(f"  - 平均: {report.throughput_stats['average']:.0f} points/sec")
    print(f"  - 峰值: {report.throughput_stats['peak']:.0f} points/sec")
    print(f"  - 最小: {report.throughput_stats['min']:.0f} points/sec")

    print(f"\n资源使用效率:")
    print(f"  - CPU效率: {report.resource_efficiency['cpu_efficiency']:.1%}")
    print(f"  - 内存效率: {report.resource_efficiency['memory_efficiency']:.1%}")
    print(f"  - 并行效率: {report.resource_efficiency['parallel_efficiency']:.1%}")
    print(f"  - 整体效率: {report.resource_efficiency['overall_efficiency']:.1%}")

    print(f"\n主要性能瓶颈:")
    for i, bottleneck in enumerate(report.bottlenecks[:5], 1):
        print(f"  {i}. {bottleneck}")

    print(f"\n优化建议 (前5项):")
    for i, recommendation in enumerate(report.optimization_recommendations[:5], 1):
        print(f"  {i}. {recommendation}")

if __name__ == "__main__":
    main()