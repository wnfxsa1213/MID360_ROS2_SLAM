"""
离线地图精细化工具包

主要功能：
- 法向量ICP增强配准算法
- 平面结构保护
- 边缘特征强化  
- 几何约束优化
- 点云去噪
- 表面重建
- 细节保护过滤
- 密度均匀化

作者：Claude Code
版本：1.0.0
"""

__version__ = "1.0.0"
__author__ = "Claude Code"

from .algorithms import *
from .refinement import *
from .utils import *