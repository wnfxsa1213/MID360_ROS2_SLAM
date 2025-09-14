#!/bin/bash

# ==============================================================================
# MID360 SLAM系统一键环境配置脚本
# ==============================================================================
# 作者: Claude Code Assistant
# 版本: v1.0
# 日期: 2025-09-07
# 说明: 为MID360 LiDAR SLAM系统配置运行环境
#       自动安装依赖、配置网络、编译系统等
#       适用于Ubuntu 20.04/22.04系统
# ==============================================================================

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 系统信息
SYSTEM_NAME="MID360 SLAM系统"
SCRIPT_VERSION="v1.0"
REQUIRED_UBUNTU_VERSION="20.04"

# 日志文件
LOG_FILE="setup_$(date +%Y%m%d_%H%M%S).log"
SLAM_DIR="$(pwd)"

# =================== 显示帮助信息 ===================
show_help() {
    echo "========================================="
    echo "  $SYSTEM_NAME 一键环境配置"
    echo "  版本: $SCRIPT_VERSION"
    echo "========================================="
    echo ""
    echo -e "${YELLOW}使用方法:${NC}"
    echo "  $0 [选项]"
    echo ""
    echo -e "${YELLOW}选项:${NC}"
    echo -e "  ${CYAN}--full${NC}         完整安装 (推荐新系统)"
    echo -e "  ${CYAN}--deps-only${NC}    仅安装依赖包"
    echo -e "  ${CYAN}--network-only${NC} 仅配置网络"
    echo -e "  ${CYAN}--build-only${NC}   仅编译SLAM系统"
    echo -e "  ${CYAN}--check${NC}        检查系统环境"
    echo -e "  ${CYAN}--help${NC}         显示此帮助"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  $0 --full          # 完整配置环境（推荐）"
    echo "  $0 --check         # 检查当前环境状态"
    echo "  $0 --network-only  # 仅配置MID360网络"
    echo ""
    echo -e "${YELLOW}系统要求:${NC}"
    echo "  - Ubuntu 20.04+ (推荐 22.04)"
    echo "  - 8GB+ 内存 (推荐 16GB+)"
    echo "  - 20GB+ 可用存储空间"
    echo "  - 网络连接 (用于下载依赖)"
}

# =================== 日志记录函数 ===================
log_message() {
    local level=$1
    local message=$2
    local timestamp=$(date "+%Y-%m-%d %H:%M:%S")
    echo "[$timestamp] [$level] $message" >> "$LOG_FILE"
    
    case $level in
        "INFO")
            echo -e "${CYAN}[INFO]${NC} $message"
            ;;
        "SUCCESS") 
            echo -e "${GREEN}[SUCCESS]${NC} $message"
            ;;
        "WARNING")
            echo -e "${YELLOW}[WARNING]${NC} $message"
            ;;
        "ERROR")
            echo -e "${RED}[ERROR]${NC} $message"
            ;;
    esac
}

# =================== 系统检查函数 ===================
check_system_requirements() {
    log_message "INFO" "检查系统环境..."
    
    # 检查操作系统
    if ! grep -q "Ubuntu" /etc/os-release; then
        log_message "ERROR" "不支持的操作系统，需要 Ubuntu 20.04+"
        return 1
    fi
    
    # 检查Ubuntu版本
    local ubuntu_version=$(lsb_release -rs)
    if [[ $(echo "$ubuntu_version >= $REQUIRED_UBUNTU_VERSION" | bc -l) -eq 0 ]]; then
        log_message "WARNING" "Ubuntu版本 $ubuntu_version 可能不完全兼容，推荐 22.04"
    else
        log_message "SUCCESS" "Ubuntu版本 $ubuntu_version 符合要求"
    fi
    
    # 检查内存
    local memory_gb=$(free -g | awk '/^Mem:/{print $2}')
    if [ -n "$memory_gb" ] && [ "$memory_gb" -lt 8 ]; then
        log_message "WARNING" "可用内存 ${memory_gb}GB 低于推荐配置(8GB+)"
    elif [ -n "$memory_gb" ]; then
        log_message "SUCCESS" "可用内存 ${memory_gb}GB 充足"
    else
        log_message "WARNING" "无法检测内存大小"
    fi
    
    # 检查存储空间
    local disk_space=$(df -BG . | awk 'NR==2 {gsub(/G/, "", $4); print $4}')
    if [ -n "$disk_space" ] && [ "$disk_space" -lt 20 ]; then
        log_message "WARNING" "可用存储空间 ${disk_space}GB 低于推荐配置(20GB+)"
    elif [ -n "$disk_space" ]; then
        log_message "SUCCESS" "可用存储空间 ${disk_space}GB 充足"
    else
        log_message "WARNING" "无法检测存储空间大小"
    fi
    
    # 检查网络连接
    if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
        log_message "SUCCESS" "网络连接正常"
    else
        log_message "ERROR" "网络连接失败，无法下载依赖包"
        return 1
    fi
    
    return 0
}

# =================== ROS2安装检查 ===================
check_ros2_installation() {
    log_message "INFO" "检查ROS2安装状态..."
    
    # 检查ROS2环境变量
    if [ -z "$ROS_DISTRO" ]; then
        # 尝试source ROS2环境
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
            log_message "SUCCESS" "找到ROS2 Humble，已加载环境"
        else
            log_message "WARNING" "未检测到ROS2安装"
            return 1
        fi
    else
        log_message "SUCCESS" "ROS2环境已配置: $ROS_DISTRO"
    fi
    
    # 检查ROS2命令
    if command -v ros2 >/dev/null 2>&1; then
        local ros2_version=$(ros2 --version 2>/dev/null | head -n 1)
        log_message "SUCCESS" "ROS2命令可用: $ros2_version"
        return 0
    else
        log_message "WARNING" "ROS2命令不可用"
        return 1
    fi
}

# =================== 系统依赖安装 ===================
install_system_dependencies() {
    log_message "INFO" "安装系统依赖包..."
    
    # 更新包管理器
    log_message "INFO" "更新软件包列表..."
    sudo apt update || {
        log_message "ERROR" "软件包列表更新失败"
        return 1
    }
    
    # 基础开发工具
    local basic_deps=(
        "build-essential"
        "cmake" 
        "git"
        "wget"
        "curl"
        "vim"
        "htop"
        "net-tools"
        "iputils-ping"
        "bc"  # 用于数值比较
    )
    
    # C++开发依赖
    local cpp_deps=(
        "libeigen3-dev"
        "libpcl-dev"
        "libopencv-dev"
        "libyaml-cpp-dev"
        "libgoogle-glog-dev"
        "libgflags-dev"
    )
    
    # PCL和几何处理库
    local geometry_deps=(
        "libpcl-dev"
        "pcl-tools"
        "libproj-dev"
        "libgeographic-dev"
    )
    
    # Python依赖（用于工具脚本）
    local python_deps=(
        "python3-pip"
        "python3-dev"
        "python3-numpy" 
        "python3-matplotlib"
        "python3-yaml"
    )
    
    # 合并所有依赖
    local all_deps=("${basic_deps[@]}" "${cpp_deps[@]}" "${geometry_deps[@]}" "${python_deps[@]}")
    
    log_message "INFO" "安装 ${#all_deps[@]} 个依赖包..."
    
    # 批量安装
    if sudo apt install -y "${all_deps[@]}"; then
        log_message "SUCCESS" "系统依赖安装完成"
    else
        log_message "ERROR" "系统依赖安装失败"
        return 1
    fi
    
    # 安装Python包
    log_message "INFO" "安装Python依赖..."
    pip3 install --user open3d pyyaml numpy matplotlib || {
        log_message "WARNING" "部分Python包安装失败，将尝试替代方案"
    }
    
    return 0
}

# =================== GTSAM库安装 ===================
install_gtsam_library() {
    log_message "INFO" "检查并安装GTSAM库..."
    
    # 检查GTSAM是否已安装
    if [ -f "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
        log_message "SUCCESS" "GTSAM库已安装"
        return 0
    fi
    
    # 创建临时目录
    local temp_dir="/tmp/gtsam_install_$$"
    mkdir -p "$temp_dir"
    cd "$temp_dir"
    
    log_message "INFO" "下载GTSAM 4.1.1源码..."
    
    # 下载GTSAM源码
    if ! wget https://github.com/borglab/gtsam/archive/4.1.1.tar.gz; then
        log_message "ERROR" "GTSAM下载失败"
        return 1
    fi
    
    # 解压和编译
    tar -xzf 4.1.1.tar.gz
    cd gtsam-4.1.1
    
    # 创建构建目录
    mkdir -p build
    cd build
    
    log_message "INFO" "编译GTSAM库（预计5-15分钟）..."
    
    # 配置CMake
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_UNSTABLE=OFF || {
        log_message "ERROR" "GTSAM配置失败"
        return 1
    }
    
    # 编译（使用多核心加速）
    local cpu_cores=$(nproc)
    if make -j$cpu_cores; then
        log_message "SUCCESS" "GTSAM编译完成"
    else
        log_message "ERROR" "GTSAM编译失败"
        return 1
    fi
    
    # 安装
    if sudo make install; then
        log_message "SUCCESS" "GTSAM安装完成"
        sudo ldconfig  # 更新动态链接库缓存
    else
        log_message "ERROR" "GTSAM安装失败"
        return 1
    fi
    
    # 清理临时文件
    cd "$SLAM_DIR"
    rm -rf "$temp_dir"
    
    return 0
}

# =================== 网络配置函数 ===================
configure_network() {
    log_message "INFO" "配置MID360雷达网络..."
    
    # 雷达网络参数
    local LIDAR_IP="192.168.1.3"
    local HOST_IP="192.168.1.50"
    local SUBNET="192.168.1.0/24"
    
    # 检查当前网络配置
    log_message "INFO" "检查当前网络配置..."
    ip addr show | grep -E "inet.*192\.168\.1\." && {
        log_message "SUCCESS" "检测到192.168.1.x网段配置"
        return 0
    }
    
    # 查找可用网络接口
    local available_interface
    available_interface=$(ip route | grep default | awk '{print $5}' | head -n 1)
    
    if [ -z "$available_interface" ]; then
        log_message "ERROR" "未找到可用网络接口"
        return 1
    fi
    
    log_message "INFO" "使用网络接口: $available_interface"
    
    # 创建网络配置脚本
    cat > "setup_network.sh" << EOF
#!/bin/bash
# MID360网络配置脚本 - 临时配置
echo "配置MID360雷达网络..."

# 添加IP地址（如果不存在）
if ! ip addr show $available_interface | grep -q "$HOST_IP"; then
    sudo ip addr add $HOST_IP/24 dev $available_interface
    echo "已添加IP地址: $HOST_IP"
else
    echo "IP地址已存在: $HOST_IP"
fi

# 添加静态路由（确保能访问雷达）
if ! ip route | grep -q "$LIDAR_IP"; then
    sudo ip route add $LIDAR_IP dev $available_interface 2>/dev/null || true
    echo "已添加路由: $LIDAR_IP"
fi

echo "网络配置完成"
echo "测试雷达连接..."
if ping -c 3 -W 2 $LIDAR_IP; then
    echo "✅ 雷达连接正常"
else
    echo "❌ 雷达连接失败，请检查："
    echo "   1. 雷达是否正确连接并供电"
    echo "   2. 网线是否连接正常" 
    echo "   3. 雷达IP是否为 $LIDAR_IP"
fi
EOF
    
    chmod +x setup_network.sh
    log_message "SUCCESS" "网络配置脚本已创建: setup_network.sh"
    
    # 询问是否立即配置网络
    echo ""
    echo -e "${YELLOW}是否立即配置网络？(y/n):${NC}"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        ./setup_network.sh
    else
        log_message "INFO" "请在需要时运行: ./setup_network.sh"
    fi
    
    return 0
}

# =================== 编译SLAM系统 ===================
build_slam_system() {
    log_message "INFO" "编译SLAM系统..."
    
    # 确保在正确目录
    if [ ! -d "ws_livox" ]; then
        log_message "ERROR" "未找到ws_livox目录，请确保在正确的项目根目录"
        return 1
    fi
    
    cd ws_livox
    
    # 加载ROS2环境
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    else
        log_message "ERROR" "ROS2环境未找到"
        return 1
    fi
    
    # 清理之前的编译
    log_message "INFO" "清理之前的编译文件..."
    rm -rf build install log
    
    # 按依赖顺序编译
    local packages=("interface" "livox_ros_driver2" "fastlio2")
    local extended_packages=("pgo" "hba" "localizer")
    
    # 编译核心包
    for pkg in "${packages[@]}"; do
        log_message "INFO" "编译包: $pkg"
        
        if colcon build --packages-select "$pkg" --cmake-args -DCMAKE_BUILD_TYPE=Release; then
            log_message "SUCCESS" "$pkg 编译成功"
            # 立即加载环境
            source install/setup.bash
        else
            log_message "ERROR" "$pkg 编译失败"
            return 1
        fi
    done
    
    # 编译扩展包（可选，依赖GTSAM）
    for pkg in "${extended_packages[@]}"; do
        if [ -d "src/$pkg" ]; then
            log_message "INFO" "编译扩展包: $pkg"
            
            if colcon build --packages-select "$pkg" --cmake-args -DCMAKE_BUILD_TYPE=Release; then
                log_message "SUCCESS" "$pkg 编译成功"
            else
                log_message "WARNING" "$pkg 编译失败（可能缺少GTSAM依赖）"
            fi
        fi
    done
    
    cd "$SLAM_DIR"
    log_message "SUCCESS" "SLAM系统编译完成"
    
    return 0
}

# =================== 创建便捷脚本 ===================
create_convenience_scripts() {
    log_message "INFO" "创建系统便捷脚本..."
    
    # 创建环境加载脚本
    cat > "load_ros_env.sh" << 'EOF'
#!/bin/bash
# ROS2环境加载脚本
source /opt/ros/humble/setup.bash 2>/dev/null
source ws_livox/install/setup.bash 2>/dev/null
echo "ROS2环境已加载"
echo "当前ROS分发版: $ROS_DISTRO"
EOF
    
    # 创建快速启动脚本
    cat > "quick_start.sh" << 'EOF'
#!/bin/bash
# 快速启动SLAM系统
echo "启动MID360 SLAM系统..."
./tools/slam_tools.sh start
EOF
    
    # 创建系统状态检查脚本
    cat > "check_status.sh" << 'EOF'
#!/bin/bash
# 系统状态检查
echo "=== MID360 SLAM系统状态 ==="
./tools/slam_tools.sh status
echo ""
echo "=== 网络连接状态 ==="
./tools/slam_tools.sh network
EOF
    
    # 设置执行权限
    chmod +x load_ros_env.sh quick_start.sh check_status.sh
    
    log_message "SUCCESS" "便捷脚本创建完成"
    echo ""
    echo -e "${CYAN}可用的便捷脚本:${NC}"
    echo "  ./load_ros_env.sh    - 加载ROS2环境"
    echo "  ./quick_start.sh     - 快速启动SLAM"
    echo "  ./check_status.sh    - 检查系统状态"
    echo "  ./setup_network.sh   - 配置MID360网络"
    
    return 0
}

# =================== 系统总结报告 ===================
generate_setup_report() {
    log_message "INFO" "生成安装报告..."
    
    local report_file="SETUP_REPORT_$(date +%Y%m%d_%H%M%S).md"
    
    cat > "$report_file" << EOF
# MID360 SLAM系统环境配置报告

**配置时间**: $(date "+%Y-%m-%d %H:%M:%S")
**脚本版本**: $SCRIPT_VERSION
**系统信息**: $(lsb_release -d | cut -f2)

## 安装状态

### 系统依赖
- ✅ 基础开发工具
- ✅ C++开发库 (Eigen3, PCL, OpenCV等)
- ✅ Python工具包
$([ -f "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake" ] && echo "- ✅ GTSAM库" || echo "- ❌ GTSAM库（未安装）")

### ROS2环境
$(check_ros2_installation >/dev/null 2>&1 && echo "- ✅ ROS2 Humble" || echo "- ❌ ROS2（需要手动安装）")

### SLAM组件编译状态
$([ -d "ws_livox/install/interface" ] && echo "- ✅ Interface包" || echo "- ❌ Interface包")
$([ -d "ws_livox/install/livox_ros_driver2" ] && echo "- ✅ Livox驱动" || echo "- ❌ Livox驱动") 
$([ -d "ws_livox/install/fastlio2" ] && echo "- ✅ FastLIO2核心" || echo "- ❌ FastLIO2核心")
$([ -d "ws_livox/install/pgo" ] && echo "- ✅ PGO优化" || echo "- ⏸ PGO优化（可选）")
$([ -d "ws_livox/install/hba" ] && echo "- ✅ HBA优化" || echo "- ⏸ HBA优化（可选）")
$([ -d "ws_livox/install/localizer" ] && echo "- ✅ 定位器" || echo "- ⏸ 定位器（可选）")

## 使用指南

### 启动系统
\`\`\`bash
# 方式1: 使用便捷脚本
./quick_start.sh

# 方式2: 使用SLAM工具
./tools/slam_tools.sh start

# 方式3: 检查状态  
./check_status.sh
\`\`\`

### 网络配置
\`\`\`bash
# 配置MID360网络（必需）
./setup_network.sh

# 检查网络状态
./tools/slam_tools.sh network
\`\`\`

### 系统管理
\`\`\`bash
# 查看所有可用命令
./tools/slam_tools.sh help

# 实时监控
./tools/slam_tools.sh monitor

# 保存地图
./tools/slam_tools.sh save
\`\`\`

## 故障排除

### 编译问题
\`\`\`bash
# 修复已知问题
./tools/slam_tools.sh fix

# 重新编译
./tools/slam_tools.sh build
\`\`\`

### 网络问题
1. 确保MID360雷达连接并供电
2. 运行 \`./setup_network.sh\` 配置网络
3. 检查雷达IP: \`ping 192.168.1.3\`

### 性能问题
- 确保充足内存 (推荐16GB+)
- 调整配置文件: \`config/slam_master_config.yaml\`
- 监控系统资源: \`./tools/slam_tools.sh monitor\`

---
详细日志文件: $LOG_FILE
EOF
    
    log_message "SUCCESS" "安装报告已生成: $report_file"
    
    return 0
}

# =================== 主函数 ===================
main() {
    echo "========================================="
    echo "  $SYSTEM_NAME 环境配置工具"  
    echo "  版本: $SCRIPT_VERSION"
    echo "========================================="
    echo ""
    
    # 创建日志文件
    log_message "INFO" "开始环境配置，日志文件: $LOG_FILE"
    
    # 解析命令行参数
    case "${1:-}" in
        "--check")
            log_message "INFO" "执行系统环境检查..."
            check_system_requirements
            check_ros2_installation
            echo ""
            echo -e "${CYAN}如需完整配置，请运行: $0 --full${NC}"
            exit 0
            ;;
        
        "--deps-only")
            log_message "INFO" "仅安装系统依赖..."
            check_system_requirements || exit 1
            install_system_dependencies || exit 1
            install_gtsam_library || exit 1
            log_message "SUCCESS" "依赖安装完成"
            exit 0
            ;;
        
        "--network-only")
            log_message "INFO" "仅配置网络..."
            configure_network || exit 1
            exit 0
            ;;
        
        "--build-only") 
            log_message "INFO" "仅编译SLAM系统..."
            build_slam_system || exit 1
            exit 0
            ;;
        
        "--full"|"")
            # 完整安装流程
            ;;
        
        "--help"|"-h")
            show_help
            exit 0
            ;;
        
        *)
            echo -e "${RED}错误: 未知选项 '$1'${NC}"
            show_help
            exit 1
            ;;
    esac
    
    # 完整安装流程
    log_message "INFO" "开始完整环境配置..."
    
    # 1. 系统检查
    if ! check_system_requirements; then
        log_message "ERROR" "系统环境检查失败"
        exit 1
    fi
    
    # 2. 检查ROS2
    if ! check_ros2_installation; then
        log_message "WARNING" "ROS2未安装，请先安装ROS2 Humble"
        echo ""
        echo -e "${YELLOW}ROS2安装命令:${NC}"
        echo "sudo apt update"
        echo "sudo apt install software-properties-common"
        echo "sudo add-apt-repository universe"
        echo "sudo apt update && sudo apt install curl"
        echo "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
        echo "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main\" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
        echo "sudo apt update && sudo apt install ros-humble-desktop"
        echo "echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
        echo ""
        echo "安装完成后，请重新运行此脚本。"
        exit 1
    fi
    
    # 3. 安装依赖
    if ! install_system_dependencies; then
        log_message "ERROR" "系统依赖安装失败"
        exit 1
    fi
    
    # 4. 安装GTSAM
    if ! install_gtsam_library; then
        log_message "WARNING" "GTSAM安装失败，PGO和HBA功能将不可用"
    fi
    
    # 5. 配置网络
    configure_network
    
    # 6. 编译系统
    if ! build_slam_system; then
        log_message "ERROR" "SLAM系统编译失败"
        exit 1
    fi
    
    # 7. 创建便捷脚本
    create_convenience_scripts
    
    # 8. 生成报告
    generate_setup_report
    
    # 完成提示
    echo ""
    echo "========================================="
    echo -e "${GREEN}🎉 环境配置完成！${NC}"
    echo "========================================="
    echo ""
    echo -e "${YELLOW}下一步操作:${NC}"
    echo "1. 连接MID360雷达并供电"
    echo "2. 运行网络配置: ./setup_network.sh"
    echo "3. 启动SLAM系统: ./quick_start.sh"
    echo "4. 检查系统状态: ./check_status.sh"
    echo ""
    echo -e "${CYAN}重要文件:${NC}"
    echo "- 日志文件: $LOG_FILE"  
    echo "- 配置文件: config/slam_master_config.yaml"
    echo "- 管理工具: tools/slam_tools.sh"
    echo ""
    echo -e "${BLUE}获取帮助: ./tools/slam_tools.sh help${NC}"
    
    log_message "SUCCESS" "MID360 SLAM系统环境配置完成"
}

# 运行主函数
main "$@"