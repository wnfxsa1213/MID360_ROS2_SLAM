# SLAM系统管理工具

本目录包含MID360 FASTLIO2_ROS2 SLAM系统的管理工具。

## 工具说明

### 主要脚本

1. **start_slam.sh** - SLAM系统启动脚本
   - 自动检查网络配置
   - 按顺序启动各个组件
   - 实时状态监控和错误处理

2. **stop_slam.sh** - 系统安全停止脚本
   - 优雅关闭所有进程
   - 清理临时文件和PID文件

3. **check_slam.sh** - 系统状态检查工具
   - 网络连接测试
   - ROS进程监控
   - 话题状态检查
   - 数据流测试

4. **slam_tools.sh** - 综合管理工具集
   - 一键式系统管理
   - 多种调试和监控功能

### 使用方法

```bash
# 基本操作
./start_slam.sh    # 启动系统
./stop_slam.sh     # 停止系统  
./check_slam.sh    # 检查状态

# 工具集使用
./slam_tools.sh help      # 查看帮助
./slam_tools.sh restart   # 重启系统
./slam_tools.sh monitor   # 监控数据流
./slam_tools.sh network   # 网络诊断
```

### 权限要求

所有脚本都需要可执行权限：
```bash
chmod +x *.sh
```