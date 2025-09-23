# FilterStats.msg - 动态对象过滤器统计信息
# 用于发布过滤器的实时性能和统计数据

# 基本统计信息
uint64 total_points          # 总点数
uint64 dynamic_points        # 动态点数
uint64 static_points         # 静态点数
float64 filter_ratio         # 过滤比例 [0, 1]

# 性能指标
float64 processing_time_ms   # 处理时间(毫秒)
uint32 history_frames        # 当前历史帧数
float64 memory_usage_mb      # 内存使用量(MB)

# 时间戳信息
builtin_interfaces/Time stamp # 消息时间戳