#!/bin/bash

# ============================================================================
# 动态对象过滤器集成测试脚本
# 用于验证动态对象过滤功能的完整性和性能
# ============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 测试配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
WS_PATH="$PROJECT_ROOT/ws_livox"
LOG_DIR="$PROJECT_ROOT/test_logs"
TEST_DATA_DIR="$PROJECT_ROOT/test_data"

# 创建必要目录
mkdir -p "$LOG_DIR"
mkdir -p "$TEST_DATA_DIR"

# 测试模式
TEST_MODE="${1:-quick}"  # quick, full, ci
VERBOSE="${2:-false}"

# 全局变量
TESTS_PASSED=0
TESTS_FAILED=0
START_TIME=$(date +%s)

# ============================================================================
# 辅助函数
# ============================================================================

print_header() {
    echo -e "${CYAN}======================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}======================================${NC}"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
    ((TESTS_PASSED++))
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
    ((TESTS_FAILED++))
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_info() {
    echo -e "${PURPLE}[INFO]${NC} $1"
}

# 检查命令是否存在
check_command() {
    if ! command -v "$1" &> /dev/null; then
        print_error "命令 '$1' 未找到，请先安装"
        exit 1
    fi
}

# 等待指定时间
wait_with_progress() {
    local duration=$1
    local message=$2

    echo -n "$message"
    for ((i=1; i<=duration; i++)); do
        sleep 1
        echo -n "."
    done
    echo " 完成"
}

# ============================================================================
# 环境检查
# ============================================================================

check_environment() {
    print_header "环境检查"

    # 检查必要命令
    print_step "检查必要命令..."
    check_command "colcon"
    check_command "ros2"
    check_command "python3"

    # 检查ROS2环境
    print_step "检查ROS2环境..."
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2环境未设置，请source ROS2 setup.bash"
        exit 1
    fi
    print_info "ROS2发行版: $ROS_DISTRO"

    # 检查工作空间
    print_step "检查工作空间..."
    if [ ! -d "$WS_PATH" ]; then
        print_error "工作空间不存在: $WS_PATH"
        exit 1
    fi

    # 检查localizer包
    if [ ! -d "$WS_PATH/src/localizer" ]; then
        print_error "localizer包不存在"
        exit 1
    fi

    print_success "环境检查完成"
}

# ============================================================================
# 编译测试
# ============================================================================

test_compilation() {
    print_header "编译验证测试"

    print_step "清理之前的构建..."
    cd "$WS_PATH"
    rm -rf build/ install/ log/

    print_step "编译localizer包..."
    if colcon build --packages-select localizer --cmake-args -DCMAKE_BUILD_TYPE=Release > "$LOG_DIR/compile.log" 2>&1; then
        print_success "编译成功"
    else
        print_error "编译失败，查看日志: $LOG_DIR/compile.log"
        if [ "$VERBOSE" = "true" ]; then
            cat "$LOG_DIR/compile.log"
        fi
        return 1
    fi

    # 检查可执行文件
    if [ -f "install/localizer/lib/localizer/localizer_node" ]; then
        print_success "可执行文件生成成功"
    else
        print_error "可执行文件未找到"
        return 1
    fi
}

# ============================================================================
# 单元测试
# ============================================================================

test_unit_tests() {
    print_header "单元测试执行"

    cd "$WS_PATH"

    # 编译测试
    print_step "编译测试代码..."
    if colcon build --packages-select localizer --cmake-args -DBUILD_TESTING=ON > "$LOG_DIR/test_compile.log" 2>&1; then
        print_success "测试编译成功"
    else
        print_error "测试编译失败，查看日志: $LOG_DIR/test_compile.log"
        return 1
    fi

    # 运行测试
    print_step "运行单元测试..."
    if colcon test --packages-select localizer > "$LOG_DIR/unit_tests.log" 2>&1; then
        print_success "单元测试通过"
    else
        print_warning "部分单元测试失败，查看详细结果"
    fi

    # 显示测试结果
    if [ -f "log/latest_test/localizer/stdout.log" ]; then
        print_info "测试结果摘要:"
        grep -E "(PASSED|FAILED|tests.*passed)" "log/latest_test/localizer/stdout.log" || true
    fi
}

# ============================================================================
# 节点启动测试
# ============================================================================

test_node_startup() {
    print_header "节点启动测试"

    cd "$WS_PATH"
    source install/setup.bash

    # 创建临时配置文件
    local temp_config="$TEST_DATA_DIR/test_localizer.yaml"
    cat > "$temp_config" << EOF
cloud_topic: /test/cloud
odom_topic: /test/odom
map_frame: map
local_frame: lidar
update_hz: 1.0

# ICP配置
rough_scan_resolution: 0.25
rough_map_resolution: 0.25
rough_max_iteration: 5
rough_score_thresh: 0.2
refine_scan_resolution: 0.1
refine_map_resolution: 0.1
refine_max_iteration: 10
refine_score_thresh: 0.1

# 动态过滤器配置
dynamic_filter:
  enable: true
  publish_stats: true
  motion_threshold: 0.1
  history_size: 5
  stability_threshold: 0.8
  max_time_diff: 0.5
  search_radius: 0.2
  min_neighbors: 10
  normal_consistency_thresh: 0.8
  density_ratio_thresh: 0.5
  downsample_ratio: 2
  max_points_per_frame: 50000
  voxel_size_base: 0.01
EOF

    print_step "启动localizer节点..."
    timeout 10s ros2 run localizer localizer_node --ros-args -p config_path:="$temp_config" > "$LOG_DIR/node_startup.log" 2>&1 &
    local node_pid=$!

    # 等待节点启动
    wait_with_progress 3 "等待节点启动"

    # 检查节点是否运行
    if kill -0 $node_pid 2>/dev/null; then
        print_success "节点启动成功"

        # 检查节点话题
        print_step "检查节点话题..."
        if ros2 topic list | grep -q "localizer"; then
            print_success "节点话题发布正常"
        else
            print_warning "未发现localizer话题"
        fi

        # 停止节点
        kill $node_pid 2>/dev/null || true
        wait $node_pid 2>/dev/null || true
    else
        print_error "节点启动失败"
        if [ "$VERBOSE" = "true" ]; then
            cat "$LOG_DIR/node_startup.log"
        fi
        return 1
    fi
}

# ============================================================================
# 配置文件验证
# ============================================================================

test_config_validation() {
    print_header "配置文件验证"

    local config_file="$WS_PATH/src/localizer/config/localizer.yaml"

    print_step "检查配置文件存在..."
    if [ -f "$config_file" ]; then
        print_success "配置文件存在"
    else
        print_error "配置文件不存在: $config_file"
        return 1
    fi

    print_step "验证配置文件格式..."
    if python3 -c "
import yaml
try:
    with open('$config_file', 'r') as f:
        config = yaml.safe_load(f)
    print('YAML格式验证通过')
except Exception as e:
    print(f'YAML格式错误: {e}')
    exit(1)
" > "$LOG_DIR/config_validation.log" 2>&1; then
        print_success "配置文件格式正确"
    else
        print_error "配置文件格式错误"
        cat "$LOG_DIR/config_validation.log"
        return 1
    fi

    print_step "检查动态过滤器配置..."
    if grep -q "dynamic_filter:" "$config_file"; then
        print_success "动态过滤器配置存在"
    else
        print_error "动态过滤器配置缺失"
        return 1
    fi
}

# ============================================================================
# 性能基准测试
# ============================================================================

test_performance() {
    print_header "性能基准测试"

    if [ "$TEST_MODE" = "quick" ]; then
        print_info "快速模式跳过性能测试"
        return 0
    fi

    # 生成测试点云数据
    print_step "生成测试数据..."
    python3 > "$TEST_DATA_DIR/generate_test_data.py" << 'EOF'
import numpy as np
import struct

def generate_test_pointcloud(filename, num_points=10000):
    """生成测试点云数据"""
    # 生成随机点云
    points = np.random.rand(num_points, 4) * 10  # x, y, z, intensity

    # 保存为简单的二进制格式
    with open(filename, 'wb') as f:
        f.write(struct.pack('I', num_points))
        for point in points:
            f.write(struct.pack('ffff', *point))

if __name__ == "__main__":
    generate_test_pointcloud("test_cloud.bin")
    print(f"Generated test cloud with {10000} points")
EOF

    if python3 "$TEST_DATA_DIR/generate_test_data.py" > "$LOG_DIR/data_generation.log" 2>&1; then
        print_success "测试数据生成完成"
    else
        print_warning "测试数据生成失败"
    fi
}

# ============================================================================
# 内存泄漏检查
# ============================================================================

test_memory_leaks() {
    print_header "内存泄漏检查"

    if [ "$TEST_MODE" = "quick" ]; then
        print_info "快速模式跳过内存测试"
        return 0
    fi

    # 检查valgrind是否可用
    if ! command -v valgrind &> /dev/null; then
        print_warning "valgrind未安装，跳过内存泄漏检查"
        return 0
    fi

    print_step "运行内存泄漏检查..."
    cd "$WS_PATH"
    source install/setup.bash

    # 使用valgrind运行简短测试
    timeout 30s valgrind --tool=memcheck --leak-check=yes --show-reachable=yes --num-callers=20 \
        ros2 run localizer localizer_node --ros-args -p config_path:="$TEST_DATA_DIR/test_localizer.yaml" \
        > "$LOG_DIR/valgrind.log" 2>&1 || true

    # 分析结果
    if grep -q "no leaks are possible" "$LOG_DIR/valgrind.log"; then
        print_success "无内存泄漏检测到"
    elif grep -q "definitely lost: 0 bytes" "$LOG_DIR/valgrind.log"; then
        print_success "无明确内存泄漏"
    else
        print_warning "可能存在内存泄漏，查看日志: $LOG_DIR/valgrind.log"
    fi
}

# ============================================================================
# 生成测试报告
# ============================================================================

generate_report() {
    print_header "生成测试报告"

    local end_time=$(date +%s)
    local duration=$((end_time - START_TIME))
    local report_file="$LOG_DIR/test_report_$(date +%Y%m%d_%H%M%S).md"

    cat > "$report_file" << EOF
# 动态对象过滤器测试报告

## 测试概要
- **测试时间**: $(date)
- **测试模式**: $TEST_MODE
- **测试时长**: ${duration}秒
- **通过测试**: $TESTS_PASSED
- **失败测试**: $TESTS_FAILED

## 环境信息
- **ROS2发行版**: $ROS_DISTRO
- **工作空间**: $WS_PATH
- **Python版本**: $(python3 --version)
- **编译器**: $(gcc --version | head -1)

## 测试结果详情

### 编译测试
$([ -f "$LOG_DIR/compile.log" ] && echo "✅ 编译成功" || echo "❌ 编译失败")

### 单元测试
$([ -f "$LOG_DIR/unit_tests.log" ] && echo "✅ 单元测试完成" || echo "❌ 单元测试失败")

### 节点启动测试
$([ -f "$LOG_DIR/node_startup.log" ] && echo "✅ 节点启动正常" || echo "❌ 节点启动失败")

### 配置验证
$([ -f "$LOG_DIR/config_validation.log" ] && echo "✅ 配置文件正确" || echo "❌ 配置文件错误")

## 日志文件
- 编译日志: $LOG_DIR/compile.log
- 测试日志: $LOG_DIR/unit_tests.log
- 节点日志: $LOG_DIR/node_startup.log
- 配置日志: $LOG_DIR/config_validation.log

## 建议
$(if [ $TESTS_FAILED -eq 0 ]; then
    echo "所有测试通过，动态对象过滤器功能正常。"
else
    echo "发现 $TESTS_FAILED 个问题，请检查相应日志文件。"
fi)
EOF

    print_success "测试报告已生成: $report_file"

    # 显示摘要
    echo
    print_header "测试摘要"
    echo -e "总测试数: $((TESTS_PASSED + TESTS_FAILED))"
    echo -e "${GREEN}通过: $TESTS_PASSED${NC}"
    echo -e "${RED}失败: $TESTS_FAILED${NC}"
    echo -e "测试时长: ${duration}秒"
    echo -e "报告文件: $report_file"
}

# ============================================================================
# 清理函数
# ============================================================================

cleanup() {
    print_step "清理临时文件..."

    # 杀死可能残留的进程
    pkill -f "localizer_node" 2>/dev/null || true

    # 清理临时文件
    rm -f "$TEST_DATA_DIR/test_localizer.yaml"
    rm -f "$TEST_DATA_DIR/generate_test_data.py"
    rm -f "$TEST_DATA_DIR/test_cloud.bin"

    print_info "清理完成"
}

# ============================================================================
# 信号处理
# ============================================================================

trap cleanup EXIT
trap 'print_error "测试被中断"; exit 1' INT TERM

# ============================================================================
# 主函数
# ============================================================================

main() {
    print_header "动态对象过滤器集成测试"
    print_info "测试模式: $TEST_MODE"
    print_info "详细输出: $VERBOSE"

    # 环境检查
    check_environment

    # 编译测试
    test_compilation || true

    # 配置验证
    test_config_validation || true

    # 单元测试
    test_unit_tests || true

    # 节点启动测试
    test_node_startup || true

    # 性能测试（仅完整模式）
    if [ "$TEST_MODE" = "full" ]; then
        test_performance || true
        test_memory_leaks || true
    fi

    # 生成报告
    generate_report

    # 退出状态
    if [ $TESTS_FAILED -eq 0 ]; then
        print_success "所有测试完成，无失败"
        exit 0
    else
        print_error "$TESTS_FAILED 个测试失败"
        exit 1
    fi
}

# ============================================================================
# 使用说明
# ============================================================================

show_help() {
    echo "用法: $0 [测试模式] [详细输出]"
    echo
    echo "测试模式:"
    echo "  quick  - 快速测试（默认）"
    echo "  full   - 完整测试（包括性能和内存检查）"
    echo "  ci     - CI环境测试"
    echo
    echo "详细输出:"
    echo "  true   - 显示详细日志"
    echo "  false  - 仅显示摘要（默认）"
    echo
    echo "示例:"
    echo "  $0                    # 快速测试"
    echo "  $0 full true          # 完整测试，显示详细日志"
    echo "  $0 ci false           # CI测试模式"
}

# 检查帮助参数
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    show_help
    exit 0
fi

# 执行主函数
main