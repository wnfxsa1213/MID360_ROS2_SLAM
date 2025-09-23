# MID360 SLAM系统功能增强开发指南

## 文档概述

本文档为MID360 SLAM系统三大功能增强提供详细的开发指南：
1. **动态对象处理** - 在Localizer中添加动态对象过滤
2. **PGO回环检测** - 集成回环检测算法，提高长距离建图精度
3. **HBA实时优化** - 实现实时层次化优化，减少建图延迟

## 目录结构

```
docs/development/
├── MID360_SLAM_Enhancement_Development_Guide.md (本文档)
├── dynamic_object_filter_guide.md      # 动态对象处理详细开发指南
├── pgo_loop_detection_guide.md         # PGO回环检测详细开发指南
├── hba_realtime_optimization_guide.md  # HBA实时优化详细开发指南
├── testing_and_validation_guide.md     # 测试与验证指南
└── api_reference.md                    # API参考文档
```

## 快速开始

### 开发环境要求

- **操作系统**: Ubuntu 20.04/22.04
- **ROS版本**: ROS2 Humble
- **编译器**: GCC 9+ (支持C++17)
- **依赖库**: PCL 1.10+, Eigen 3.3+, OpenCV 4.2+, yaml-cpp

### 代码规范

本项目遵循以下编码规范：
- **C++**: Google C++ Style Guide 2024
- **Python**: PEP 8
- **ROS2**: ROS2官方开发规范
- **CMake**: 小写命令，snake_case标识符

详细规范参见 `CLAUDE.md` 文件。

## 开发优先级和时间计划

### 第一阶段：动态对象处理 (优先级1)
- **时间**: 4周
- **重要性**: 最高 - 该功能完全缺失，严重影响SLAM精度
- **详细指南**: [dynamic_object_filter_guide.md](./dynamic_object_filter_guide.md)

### 第二阶段：PGO回环检测改进 (优先级2)
- **时间**: 3周
- **重要性**: 中等 - 已有基础框架，主要改进算法
- **详细指南**: [pgo_loop_detection_guide.md](./pgo_loop_detection_guide.md)

### 第三阶段：HBA实时优化 (优先级3)
- **时间**: 4周
- **重要性**: 中等 - 需要架构重构，实现复杂
- **详细指南**: [hba_realtime_optimization_guide.md](./hba_realtime_optimization_guide.md)

## 项目架构概览

### 当前组件状态

| 组件 | 位置 | 状态 | 需要改进 |
|------|------|------|----------|
| FAST-LIO2 | `ws_livox/src/fastlio2/` | ✅ 完整 | 无 |
| PGO | `ws_livox/src/pgo/` | 🔄 基础框架 | 算法改进 |
| HBA | `ws_livox/src/hba/` | 🔄 批处理模式 | 实时架构 |
| Localizer | `ws_livox/src/localizer/` | ❌ 缺少动态过滤 | 完整实现 |
| Interface | `ws_livox/src/interface/` | ✅ 完整 | 可能需要新增服务 |

### 关键文件位置

**核心节点文件:**
- `ws_livox/src/fastlio2/src/lio_node.cpp` - SLAM核心节点
- `ws_livox/src/pgo/src/pgo_node.cpp` - PGO回环检测节点
- `ws_livox/src/hba/src/hba_node.cpp` - HBA优化节点
- `ws_livox/src/localizer/src/localizer_node.cpp` - 定位节点

**配置文件:**
- `config/launch_config.yaml` - 系统启动配置
- `ws_livox/src/fastlio2/config/lio.yaml` - SLAM算法配置
- `ws_livox/src/pgo/config/pgo.yaml` - PGO配置
- `ws_livox/src/hba/config/hba.yaml` - HBA配置
- `ws_livox/src/localizer/config/localizer.yaml` - 定位器配置

## 开发工具和脚本

### SLAM工具集
```bash
# 使用统一的SLAM工具集进行开发和测试
./tools/slam_tools.sh help     # 查看所有可用命令
./tools/slam_tools.sh build    # 编译项目
./tools/slam_tools.sh start    # 启动系统
./tools/slam_tools.sh monitor  # 监控数据流
./tools/slam_tools.sh test     # 运行测试
```

### 编译顺序
```bash
# 必须按依赖顺序编译
cd ws_livox
colcon build --packages-select interface
colcon build --packages-select livox_ros_driver2
colcon build --packages-select fastlio2
colcon build --packages-select pgo
colcon build --packages-select hba
colcon build --packages-select localizer
```

## 测试和验证

### 单元测试
每个功能模块都应该包含完整的单元测试：
```bash
# 运行特定模块的测试
colcon test --packages-select <package_name>
```

### 集成测试
```bash
# 运行完整的系统集成测试
./tools/slam_tools.sh test --integration
```

### 性能测试
```bash
# 运行性能基准测试
./tools/slam_tools.sh benchmark
```

详细的测试指南参见 [testing_and_validation_guide.md](./testing_and_validation_guide.md)

## 代码贡献流程

1. **创建功能分支**: `git checkout -b feature/dynamic-object-filter`
2. **遵循编码规范**: 参考 `CLAUDE.md` 中的规范要求
3. **编写测试**: 为新功能编写完整的测试用例
4. **文档更新**: 更新相关的技术文档
5. **性能验证**: 运行性能测试确保无回归
6. **代码审查**: 提交前进行代码审查

## 调试和故障排除

### 常用调试命令
```bash
# 查看系统状态
./tools/slam_tools.sh status

# 实时监控数据流
./tools/slam_tools.sh monitor

# 查看日志
./tools/slam_tools.sh log

# 网络诊断
./tools/slam_tools.sh network
```

### 常见问题
1. **编译错误**: 检查依赖安装和编译顺序
2. **运行时错误**: 检查网络配置和硬件连接
3. **性能问题**: 使用性能分析工具定位瓶颈

## 文档维护

本开发指南应随着项目演进持续更新：
- 新功能实现后更新相关章节
- 定期检查和更新API参考
- 添加新的最佳实践和经验总结

## 联系和支持

- **项目维护者**: 开发团队
- **技术文档**: `docs/` 目录
- **配置管理**: `config/` 目录
- **工具脚本**: `tools/` 目录

---

## 下一步行动

开始开发前，请仔细阅读对应功能的详细开发指南：

1. 🚀 **立即开始**: [动态对象处理开发指南](./dynamic_object_filter_guide.md)
2. 📋 **后续计划**: [PGO回环检测开发指南](./pgo_loop_detection_guide.md)
3. 🔄 **最终阶段**: [HBA实时优化开发指南](./hba_realtime_optimization_guide.md)

每个指南都包含详细的代码示例、实施步骤和测试方法。