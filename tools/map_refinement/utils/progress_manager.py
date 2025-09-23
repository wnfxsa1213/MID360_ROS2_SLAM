"""
统一进度管理系统

为地图精细化工具的所有模块提供统一的进度条和百分比显示功能。
支持嵌套进度、多线程安全、以及自定义样式。
"""

import threading
import time
from typing import Optional, Dict, Any, Callable
from contextlib import contextmanager
import logging

try:
    from tqdm import tqdm
    TQDM_AVAILABLE = True
except ImportError:
    TQDM_AVAILABLE = False
    print("警告: tqdm库未安装，将使用简单进度显示")

logger = logging.getLogger(__name__)


class ProgressManager:
    """统一进度管理器"""

    def __init__(self, enable_progress: bool = True, use_tqdm: bool = True):
        """
        初始化进度管理器

        Args:
            enable_progress: 是否启用进度显示
            use_tqdm: 是否使用tqdm进度条（需要安装tqdm）
        """
        self.enable_progress = enable_progress
        self.use_tqdm = use_tqdm and TQDM_AVAILABLE
        self._lock = threading.Lock()
        self._active_bars: Dict[str, Any] = {}
        self._progress_stack = []

        if not TQDM_AVAILABLE and use_tqdm:
            logger.warning("tqdm未安装，将使用简单进度显示")
            self.use_tqdm = False

    @contextmanager
    def progress_bar(self, total: int, desc: str, unit: str = "it",
                    leave: bool = True, position: Optional[int] = None):
        """
        创建进度条上下文管理器

        Args:
            total: 总步数
            desc: 描述文本
            unit: 单位名称
            leave: 完成后是否保留进度条
            position: 进度条位置（用于嵌套显示）
        """
        if not self.enable_progress:
            yield DummyProgressBar()
            return

        bar_id = f"{desc}_{int(time.time() * 1000)}"

        try:
            if self.use_tqdm:
                # 使用tqdm进度条
                bar = tqdm(
                    total=total,
                    desc=desc,
                    unit=unit,
                    leave=leave,
                    position=position,
                    dynamic_ncols=True,
                    bar_format='{desc}: {percentage:3.0f}%|{bar}| {n_fmt}/{total_fmt} [{elapsed}<{remaining}, {rate_fmt}]'
                )
            else:
                # 使用简单进度条
                bar = SimpleProgressBar(total, desc, unit)

            with self._lock:
                self._active_bars[bar_id] = bar

            yield bar

        finally:
            with self._lock:
                if bar_id in self._active_bars:
                    if hasattr(self._active_bars[bar_id], 'close'):
                        self._active_bars[bar_id].close()
                    del self._active_bars[bar_id]

    @contextmanager
    def nested_progress(self, total: int, desc: str, parent_step: float = 1.0):
        """
        创建嵌套进度条

        Args:
            total: 子任务总数
            desc: 子任务描述
            parent_step: 在父进度条中占的步数
        """
        position = len(self._progress_stack)
        self._progress_stack.append(desc)

        try:
            with self.progress_bar(total, desc, position=position, leave=False) as bar:
                yield bar
        finally:
            self._progress_stack.pop()

    def close_all(self):
        """关闭所有活跃的进度条"""
        with self._lock:
            for bar in self._active_bars.values():
                if hasattr(bar, 'close'):
                    bar.close()
            self._active_bars.clear()


class SimpleProgressBar:
    """简单进度条实现（不依赖tqdm）"""

    def __init__(self, total: int, desc: str, unit: str = "it"):
        self.total = total
        self.desc = desc
        self.unit = unit
        self.current = 0
        self.start_time = time.time()
        self._last_print_time = 0
        self._last_progress = -1

    def update(self, n: int = 1):
        """更新进度"""
        self.current = min(self.current + n, self.total)
        current_time = time.time()

        # 限制更新频率，避免输出过于频繁
        if current_time - self._last_print_time > 0.1:  # 每100ms最多更新一次
            self._print_progress()
            self._last_print_time = current_time

    def _print_progress(self):
        """打印进度信息"""
        if self.total <= 0:
            return

        progress = int(100 * self.current / self.total)

        # 只在进度变化时打印
        if progress != self._last_progress:
            elapsed = time.time() - self.start_time
            if self.current > 0:
                rate = self.current / elapsed
                eta = (self.total - self.current) / rate if rate > 0 else 0
                eta_str = f"{eta:.1f}s"
            else:
                eta_str = "?s"

            # 创建进度条
            bar_length = 30
            filled_length = int(bar_length * self.current / self.total)
            bar = '█' * filled_length + '░' * (bar_length - filled_length)

            print(f"\r{self.desc}: {progress:3d}%|{bar}| {self.current}/{self.total} "
                  f"[{elapsed:.1f}s<{eta_str}, {rate:.1f}{self.unit}/s]", end='', flush=True)

            self._last_progress = progress

    def close(self):
        """完成进度条"""
        if self.current < self.total:
            self.current = self.total
        self._print_progress()
        print()  # 换行

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


class DummyProgressBar:
    """占位进度条（当禁用进度显示时使用）"""

    def update(self, n: int = 1):
        pass

    def close(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass


# 全局进度管理器实例
_global_progress_manager: Optional[ProgressManager] = None


def initialize_progress_manager(enable_progress: bool = True, use_tqdm: bool = True) -> ProgressManager:
    """
    初始化全局进度管理器

    Args:
        enable_progress: 是否启用进度显示
        use_tqdm: 是否使用tqdm（如果可用）

    Returns:
        进度管理器实例
    """
    global _global_progress_manager
    _global_progress_manager = ProgressManager(enable_progress, use_tqdm)
    return _global_progress_manager


def get_progress_manager() -> ProgressManager:
    """获取全局进度管理器"""
    global _global_progress_manager
    if _global_progress_manager is None:
        _global_progress_manager = ProgressManager()
    return _global_progress_manager


def progress_bar(total: int, desc: str, **kwargs):
    """便捷函数：创建进度条"""
    return get_progress_manager().progress_bar(total, desc, **kwargs)


def nested_progress(total: int, desc: str, **kwargs):
    """便捷函数：创建嵌套进度条"""
    return get_progress_manager().nested_progress(total, desc, **kwargs)


# 装饰器版本
def track_progress(desc: str, unit: str = "it"):
    """
    进度跟踪装饰器

    Args:
        desc: 进度描述
        unit: 单位名称
    """
    def decorator(func: Callable):
        def wrapper(*args, **kwargs):
            # 尝试从参数中获取总数
            total = kwargs.get('total', 100)

            with progress_bar(total, desc, unit=unit) as pbar:
                # 将进度条传递给函数
                if 'progress_callback' not in kwargs:
                    kwargs['progress_callback'] = pbar.update
                return func(*args, **kwargs)
        return wrapper
    return decorator


if __name__ == "__main__":
    # 测试代码
    import time

    # 测试简单进度条
    print("测试简单进度条:")
    with progress_bar(100, "测试任务") as pbar:
        for i in range(100):
            time.sleep(0.01)
            pbar.update(1)

    # 测试嵌套进度条
    print("\n测试嵌套进度条:")
    pm = get_progress_manager()
    with pm.progress_bar(3, "主任务") as main_bar:
        for i in range(3):
            with pm.nested_progress(10, f"子任务 {i+1}") as sub_bar:
                for j in range(10):
                    time.sleep(0.05)
                    sub_bar.update(1)
            main_bar.update(1)