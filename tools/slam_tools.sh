#!/bin/bash

# FASTLIO2_ROS2 SLAM工具集合脚本
# 作者: Claude Code Assistant
# 日期: 2025-09-03
# 描述: SLAM系统管理和调试工具集合

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

show_help() {
    echo "========================================="
    echo "  MID360 FASTLIO2_ROS2 SLAM工具集"
    echo "========================================="
    echo ""
    echo -e "${YELLOW}使用方法:${NC}"
    echo "  $0 [命令]"
    echo ""
    echo -e "${YELLOW}可用命令:${NC}"
    echo -e "  ${CYAN}start${NC}     - 启动完整SLAM系统 (所有模块，仅PGO界面显示)"
    echo -e "  ${CYAN}start-basic${NC} - 仅启动基础SLAM (fastlio2)"
    echo -e "  ${CYAN}stop${NC}      - 停止SLAM系统"
    echo -e "  ${CYAN}status${NC}    - 检查系统状态"
    echo -e "  ${CYAN}restart${NC}   - 重启SLAM系统"
    echo -e "  ${CYAN}monitor${NC}   - 实时监控数据流"
    echo -e "  ${CYAN}topics${NC}    - 显示所有话题"
    echo -e "  ${CYAN}nodes${NC}     - 显示所有节点"
    echo -e "  ${CYAN}build${NC}     - 编译所有SLAM组件（完整系统）"
    echo -e "  ${CYAN}clean${NC}     - 清理编译文件"
    echo -e "  ${CYAN}fix${NC}       - 修复已知编译问题"
    echo -e "  ${CYAN}config${NC}    - 显示配置信息"
    echo -e "  ${CYAN}network${NC}   - 检查网络配置"
    echo -e "  ${CYAN}log${NC}       - 查看系统日志"
    echo -e "  ${CYAN}rviz${NC}      - 启动RViz可视化"
    echo -e "  ${CYAN}save${NC}      - 保存当前SLAM地图和轨迹"
    echo -e "  ${CYAN}view${NC}      - 查看已保存的地图文件"
    echo -e "  ${CYAN}maps${NC}      - 管理已保存的地图"
    echo -e "  ${CYAN}gtsam${NC}     - 安装GTSAM库（用于pgo和hba）"
    echo -e "  ${CYAN}deps${NC}      - 检查依赖包状态"
    echo -e "  ${CYAN}help${NC}      - 显示此帮助信息"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  $0 fix          # 修复编译问题"
    echo "  $0 build        # 重新编译系统"
    echo "  $0 start        # 启动SLAM系统"
    echo "  $0 status       # 检查运行状态"
}

# 确保在正确的目录
cd "/home/tianyu/codes/Mid360map" 2>/dev/null || {
    echo -e "${RED}错误: 无法进入SLAM系统目录${NC}"
    exit 1
}

# 加载ROS2环境
load_ros_env() {
    source /opt/ros/humble/setup.bash 2>/dev/null
    source ws_livox/install/setup.bash 2>/dev/null
}

# 启动完整SLAM系统
start_full_slam_system() {
    echo -e "${BLUE}=== 启动完整SLAM系统（统一界面） ===${NC}"
    
    # 检查依赖
    if ! check_slam_dependencies; then
        echo -e "${RED}❌ 依赖检查失败，无法启动完整系统${NC}"
        echo -e "${YELLOW}尝试运行：$0 deps 检查缺失的依赖${NC}"
        return 1
    fi
    
    load_ros_env
    
    # 使用统一launch文件启动所有组件
    echo -e "${CYAN}启动统一SLAM系统（所有组件 + 单一RViz界面）...${NC}"
    echo -e "${YELLOW}包含组件：Livox驱动 + FastLIO2 + PGO + HBA + Localizer${NC}"
    echo -e "${YELLOW}显示界面：仅PGO RViz窗口（完整信息展示）${NC}"
    
    ros2 launch fastlio2 full_slam_system.launch.py &
    SLAM_SYSTEM_PID=$!
    
    echo -e "${CYAN}等待系统启动...${NC}"
    sleep 8
    
    # 检查核心组件启动状态
    local success=0
    local total=0
    
    # 检查各个节点
    components=("lio_node:FastLIO2" "pgo_node:PGO" "hba_node:HBA" "localizer_node:Localizer" "rviz2:可视化")
    
    for component in "${components[@]}"; do
        node_name="${component%%:*}"
        display_name="${component##*:}"
        total=$((total + 1))
        
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            echo -e "${GREEN}✅ $display_name 启动成功${NC}"
            success=$((success + 1))
        else
            echo -e "${RED}❌ $display_name 启动失败${NC}"
        fi
    done
    
    echo ""
    if [ $success -eq $total ]; then
        echo -e "${GREEN}🎉 完整SLAM系统启动成功！($success/$total)${NC}"
        echo -e "${BLUE}📊 主监控窗口：PGO RViz（显示实时建图+回环检测+轨迹优化）${NC}"
    elif [ $success -gt 0 ]; then
        echo -e "${YELLOW}⚠️  部分组件启动成功 ($success/$total)${NC}"
        echo -e "${BLUE}📊 主监控窗口：PGO RViz${NC}"
    else
        echo -e "${RED}❌ 系统启动失败${NC}"
        return 1
    fi
    
    echo ""
    echo -e "${BLUE}管理命令:${NC}"
    echo -e "${BLUE}  状态检查: $0 status${NC}"
    echo -e "${BLUE}  实时监控: $0 monitor${NC}"
    echo -e "${BLUE}  保存地图: $0 save${NC}"
}

# 停止所有SLAM组件
stop_all_slam_components() {
    echo -e "${YELLOW}停止所有SLAM组件...${NC}"
    
    # 停止ROS节点
    slam_nodes=("lio_node" "pgo_node" "hba_node" "localizer_node")
    for node in "${slam_nodes[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            echo -e "${YELLOW}停止 $node...${NC}"
            ros2 lifecycle set /$node shutdown 2>/dev/null || true
        fi
    done
    
    # 强制杀死相关进程
    pkill -f "ros2 launch.*fastlio2" 2>/dev/null || true
    pkill -f "ros2 launch.*pgo" 2>/dev/null || true  
    pkill -f "ros2 launch.*hba" 2>/dev/null || true
    pkill -f "ros2 launch.*localizer" 2>/dev/null || true
    pkill -f "lio_node" 2>/dev/null || true
    pkill -f "pgo_node" 2>/dev/null || true
    pkill -f "hba_node" 2>/dev/null || true
    pkill -f "localizer_node" 2>/dev/null || true
    
    # 调用原有停止脚本
    if [ -f "tools/stop_slam.sh" ]; then
        ./tools/stop_slam.sh > /dev/null 2>&1
    fi
    
    sleep 2
    echo -e "${GREEN}✅ 所有SLAM组件已停止${NC}"
}

# 检查完整SLAM状态
check_full_slam_status() {
    echo -e "${CYAN}完整SLAM系统状态检查${NC}"
    echo "=================================="
    
    load_ros_env
    
    # 检查各个节点状态
    slam_components=(
        "lio_node:FastLIO2核心"
        "pgo_node:位姿图优化"
        "hba_node:分层束调整" 
        "localizer_node:定位器"
    )
    
    active_count=0
    total_expected=0
    
    for component in "${slam_components[@]}"; do
        node_name="${component%%:*}"
        description="${component##*:}"
        
        # 检查是否应该运行（包是否已编译）
        pkg_name=""
        case "$node_name" in
            "lio_node") pkg_name="fastlio2" ;;
            "pgo_node") pkg_name="pgo" ;;
            "hba_node") pkg_name="hba" ;;
            "localizer_node") pkg_name="localizer" ;;
        esac
        
        if [ -d "ws_livox/install/$pkg_name" ]; then
            total_expected=$((total_expected + 1))
            if ros2 node list 2>/dev/null | grep -q "$node_name"; then
                echo -e "  $description: ${GREEN}✅ 运行中${NC}"
                active_count=$((active_count + 1))
            else
                echo -e "  $description: ${RED}❌ 未运行${NC}"
            fi
        else
            echo -e "  $description: ${YELLOW}⏸  未编译${NC}"
        fi
    done
    
    echo ""
    echo -e "${YELLOW}系统状态摘要:${NC}"
    echo "  活跃组件: $active_count/$total_expected"
    
    if [ $active_count -eq $total_expected ] && [ $total_expected -gt 0 ]; then
        echo -e "  整体状态: ${GREEN}✅ 系统正常运行${NC}"
    elif [ $active_count -gt 0 ]; then
        echo -e "  整体状态: ${YELLOW}⚠️  部分组件运行${NC}"
    else
        echo -e "  整体状态: ${RED}❌ 系统未运行${NC}"
    fi
    
    # 检查话题活跃度
    echo ""
    echo -e "${YELLOW}关键话题状态:${NC}"
    key_topics=(
        "/livox/lidar:雷达数据"
        "/livox/imu:IMU数据"
        "/fastlio2/lio_odom:里程计"
        "/fastlio2/body_cloud:点云"
    )
    
    for topic_info in "${key_topics[@]}"; do
        topic="${topic_info%%:*}"
        description="${topic_info##*:}"
        
        if timeout 2s ros2 topic hz "$topic" --once 2>/dev/null | grep -q "Hz"; then
            echo -e "  $description: ${GREEN}✅ 数据流正常${NC}"
        else
            echo -e "  $description: ${RED}❌ 无数据流${NC}"
        fi
    done
}

# 检查SLAM依赖
check_slam_dependencies() {
    # 检查基础依赖 
    if ! [ -d "ws_livox/install/fastlio2" ]; then
        echo -e "${RED}❌ FastLIO2未编译${NC}"
        return 1
    fi
    
    if ! [ -d "ws_livox/install/livox_ros_driver2" ]; then
        echo -e "${RED}❌ Livox驱动未编译${NC}" 
        return 1
    fi
    
    # 检查GTSAM (PGO和HBA需要)
    if [ -d "ws_livox/src/pgo" ] || [ -d "ws_livox/src/hba" ]; then
        if [ ! -f "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
            echo -e "${YELLOW}⚠️  GTSAM未安装，PGO和HBA功能将不可用${NC}"
            echo -e "${YELLOW}   运行 '$0 gtsam' 安装GTSAM库${NC}"
        fi
    fi
    
    return 0
}

case "$1" in
    "start")
        echo -e "${GREEN}启动完整SLAM系统 (FastLIO2 + PGO + HBA + Localizer)...${NC}"
        start_full_slam_system
        ;;
    
    "start-basic")
        echo -e "${GREEN}启动基础SLAM系统 (仅FastLIO2)...${NC}"
        ./tools/start_slam.sh
        ;;
    
    "stop")
        echo -e "${YELLOW}停止SLAM系统...${NC}"
        stop_all_slam_components
        ;;
    
    "status")
        check_full_slam_status
        ;;
    
    "restart")
        echo -e "${YELLOW}重启完整SLAM系统...${NC}"
        stop_all_slam_components
        sleep 3
        start_full_slam_system
        ;;
    
    "monitor")
        echo -e "${CYAN}实时监控数据流 (Ctrl+C退出)...${NC}"
        echo ""
        load_ros_env
        
        echo -e "${YELLOW}监控里程计数据:${NC}"
        timeout 5s ros2 topic echo /fastlio2/lio_odom --once 2>/dev/null || echo "无数据"
        echo ""
        
        echo -e "${YELLOW}监控话题频率:${NC}"
        echo "IMU: $(timeout 3s ros2 topic hz /livox/imu 2>/dev/null | grep "average rate" | tail -1 || echo "无数据")"
        echo "点云: $(timeout 3s ros2 topic hz /livox/lidar 2>/dev/null | grep "average rate" | tail -1 || echo "无数据")"
        echo "里程计: $(timeout 3s ros2 topic hz /fastlio2/lio_odom 2>/dev/null | grep "average rate" | tail -1 || echo "无数据")"
        ;;
    
    "topics")
        echo -e "${CYAN}当前活跃话题:${NC}"
        load_ros_env
        ros2 topic list | sort
        ;;
    
    "nodes")
        echo -e "${CYAN}当前运行节点:${NC}"
        load_ros_env
        ros2 node list | sort
        ;;
    
    "build")
        echo -e "${YELLOW}编译完整SLAM系统（包含所有组件）...${NC}"
        cd ws_livox
        
        # 清理之前的编译结果以确保干净编译
        echo "清理之前的编译文件..."
        rm -rf build install log
        
        # 检查GTSAM依赖
        if [ ! -f "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
            echo -e "${RED}❌ GTSAM未安装，无法编译PGO和HBA包${NC}"
            echo -e "${YELLOW}请先运行: $0 gtsam${NC}"
            exit 1
        fi
        
        # 按依赖顺序编译所有包
        echo "正在编译所有SLAM组件..."
        
        # 核心包（必需）
        core_packages=("interface" "livox_ros_driver2" "fastlio2")
        extended_packages=("pgo" "hba" "localizer")
        
        # 编译核心包
        for i in "${!core_packages[@]}"; do
            pkg="${core_packages[$i]}"
            step=$((i + 1))
            echo -e "${CYAN}[${step}/6] 编译 ${pkg}...${NC}"
            colcon build --packages-select "${pkg}" --cmake-args -DCMAKE_BUILD_TYPE=Release
            if [ $? -ne 0 ]; then
                echo -e "${RED}❌ ${pkg}编译失败${NC}"
                echo -e "${YELLOW}提示：可能存在依赖问题，请检查错误信息${NC}"
                exit 1
            fi
            # 每次编译后立即source环境，确保后续包能找到依赖
            source install/setup.bash
            echo -e "${GREEN}✓ ${pkg}编译成功${NC}"
        done
        
        # 编译扩展包
        for i in "${!extended_packages[@]}"; do
            pkg="${extended_packages[$i]}"
            step=$((i + 4))  # 继续计数
            echo -e "${CYAN}[${step}/6] 编译 ${pkg}...${NC}"
            colcon build --packages-select "${pkg}" --cmake-args -DCMAKE_BUILD_TYPE=Release
            if [ $? -ne 0 ]; then
                echo -e "${YELLOW}⚠️  ${pkg}编译失败，但不影响基本功能${NC}"
            else
                echo -e "${GREEN}✓ ${pkg}编译成功${NC}"
            fi
        done
        
        echo -e "${GREEN}🎉 完整SLAM系统编译完成！${NC}"
        echo ""
        echo -e "${YELLOW}完整编译结果验证:${NC}"
        
        # 验证所有包
        all_packages=("interface" "livox_ros_driver2" "fastlio2" "pgo" "hba" "localizer")
        for pkg in "${all_packages[@]}"; do
            if [ -d "install/${pkg}" ]; then
                echo -e "  ${pkg}: ${GREEN}✓${NC} 已安装"
            else
                echo -e "  ${pkg}: ${RED}✗${NC} 未安装"
            fi
        done
        
        echo ""
        echo -e "${BLUE}🚀 现在可以启动完整SLAM系统: $0 start${NC}"
        ;;
    
    "clean")
        echo -e "${YELLOW}清理编译文件...${NC}"
        cd ws_livox
        rm -rf build install log
        echo "编译文件已清理"
        ;;
    
    "config")
        echo -e "${CYAN}系统配置信息:${NC}"
        echo "================================="
        echo "工作目录: $(pwd)"
        echo "ROS版本: $(ros2 --version 2>/dev/null || echo '未加载')"
        
        CONFIG_FILE="ws_livox/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360_config.json"
        if [ -f "$CONFIG_FILE" ]; then
            echo ""
            echo "MID360配置:"
            echo "  电脑IP: $(grep -o '192\.168\.1\.[0-9]*' "$CONFIG_FILE" | head -1)"
            echo "  雷达IP: $(grep -o '192\.168\.1\.[0-9]*' "$CONFIG_FILE" | tail -1)"
        fi
        
        FASTLIO_CONFIG="ws_livox/install/fastlio2/share/fastlio2/config/lio.yaml"
        if [ -f "$FASTLIO_CONFIG" ]; then
            echo ""
            echo "FASTLIO2配置:"
            echo "  IMU话题: $(grep 'imu_topic:' "$FASTLIO_CONFIG" | awk '{print $2}')"
            echo "  点云话题: $(grep 'lidar_topic:' "$FASTLIO_CONFIG" | awk '{print $2}')"
            echo "  扫描分辨率: $(grep 'scan_resolution:' "$FASTLIO_CONFIG" | awk '{print $2}')"
        fi
        ;;
    
    "network")
        echo -e "${CYAN}网络配置检查:${NC}"
        echo "================================="
        
        echo "本机网络接口:"
        ip addr show | grep -A 2 "state UP" | grep inet
        
        echo ""
        echo "路由表:"
        ip route | head -5
        
        echo ""
        echo "雷达连接测试:"
        if ping -c 2 -W 2 192.168.1.3 >/dev/null 2>&1; then
            echo -e "192.168.1.3: ${GREEN}在线${NC}"
        else
            echo -e "192.168.1.3: ${RED}离线${NC}"
        fi
        ;;
    
    "log")
        echo -e "${CYAN}查看系统日志:${NC}"
        LOG_DIR="$HOME/.ros/log"
        if [ -d "$LOG_DIR" ]; then
            echo "最新日志目录:"
            ls -lt "$LOG_DIR" | head -3
            
            echo ""
            echo "最新错误信息:"
            find "$LOG_DIR" -name "*.log" -mmin -10 -exec grep -l "ERROR\|FATAL\|Failed" {} \; | head -3 | while read logfile; do
                echo "=== $logfile ==="
                tail -10 "$logfile" | grep -E "ERROR|FATAL|Failed" || echo "无错误信息"
                echo ""
            done
        else
            echo "未找到ROS日志目录"
        fi
        ;;
    
    "rviz")
        echo -e "${CYAN}启动RViz可视化...${NC}"
        load_ros_env
        
        RVIZ_CONFIG="ws_livox/install/fastlio2/share/fastlio2/rviz/fastlio2.rviz"
        if [ -f "$RVIZ_CONFIG" ]; then
            echo "使用FASTLIO2配置文件启动RViz..."
            rviz2 -d "$RVIZ_CONFIG" &
        else
            echo "启动默认RViz..."
            rviz2 &
        fi
        
        echo "RViz已在后台启动"
        ;;
    
    "save")
        echo -e "${YELLOW}保存SLAM地图和轨迹...${NC}"
        
        # 检查ROS环境
        load_ros_env
        
        # 检查SLAM节点是否运行
        if ! ros2 node list | grep -q "lio_node"; then
            echo -e "${RED}❌ FAST-LIO2节点未运行${NC}"
            echo "请先运行: $0 start"
            exit 1
        fi
        
        # 确保saved_maps目录存在
        mkdir -p saved_maps
        
        # 使用简化保存脚本
        if [ -f "tools/save_map_simple.py" ]; then
            echo -e "${CYAN}调用地图保存工具...${NC}"
            cd tools
            python3 save_map_simple.py ../saved_maps
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}✅ 地图保存完成${NC}"
                echo -e "${BLUE}保存位置: $(pwd)/../saved_maps${NC}"
                # 列出保存的文件
                echo -e "${CYAN}保存的文件:${NC}"
                ls -la ../saved_maps/*$(date +%Y%m%d)* 2>/dev/null | tail -5
            else
                echo -e "${RED}❌ 地图保存失败${NC}"
            fi
        else
            echo -e "${RED}❌ 地图保存工具不存在${NC}"
            echo "请确保 tools/save_map_simple.py 文件存在"
        fi
        ;;
    
    "view")
        if [ -z "$2" ]; then
            echo -e "${YELLOW}查看可用的地图文件:${NC}"
            if [ -d "saved_maps" ]; then
                echo "saved_maps 目录中的文件:"
                ls -la saved_maps/ 2>/dev/null
                echo ""
                echo -e "${CYAN}使用方法: $0 view <点云文件路径> [轨迹文件路径]${NC}"
                echo "示例: $0 view saved_maps/mid360_map_20250904_143000.pcd"
            else
                echo -e "${RED}❌ saved_maps目录不存在${NC}"
                echo "请先运行 '$0 save' 保存地图"
            fi
        else
            echo -e "${CYAN}启动地图查看器...${NC}"
            if [ -f "tools/view_saved_map.py" ]; then
                TRAJECTORY_ARG=""
                if [ -n "$3" ] && [ -f "$3" ]; then
                    TRAJECTORY_ARG="-t $3"
                fi
                python3 tools/view_saved_map.py "$2" $TRAJECTORY_ARG
            else
                echo -e "${RED}❌ 地图查看工具不存在${NC}"
                echo "请确保 tools/view_saved_map.py 文件存在"
            fi
        fi
        ;;
    
    "maps")
        echo -e "${CYAN}地图管理工具${NC}"
        echo "=========================="
        
        if [ -d "saved_maps" ]; then
            echo -e "${YELLOW}已保存的地图:${NC}"
            
            # 统计文件
            pcd_count=$(find saved_maps -name "*.pcd" 2>/dev/null | wc -l)
            ply_count=$(find saved_maps -name "*.ply" 2>/dev/null | wc -l)
            txt_count=$(find saved_maps -name "*trajectory*.txt" 2>/dev/null | wc -l)
            
            echo "  点云地图 (.pcd): $pcd_count 个"
            echo "  点云地图 (.ply): $ply_count 个" 
            echo "  轨迹文件 (.txt): $txt_count 个"
            echo ""
            
            if [ $pcd_count -gt 0 ] || [ $ply_count -gt 0 ]; then
                echo -e "${YELLOW}最近的地图文件:${NC}"
                find saved_maps \( -name "*.pcd" -o -name "*.ply" \) -exec ls -lt {} + 2>/dev/null | head -5
                echo ""
            fi
            
            # 目录大小
            total_size=$(du -sh saved_maps 2>/dev/null | cut -f1)
            echo "总占用空间: $total_size"
            
        else
            echo -e "${RED}❌ saved_maps目录不存在${NC}"
            echo "请先运行 '$0 save' 保存地图"
        fi
        
        echo ""
        echo -e "${CYAN}常用操作:${NC}"
        echo "  $0 save                    # 保存当前地图"
        echo "  $0 view <地图文件>         # 查看地图"
        echo "  $0 maps                    # 显示此信息"
        ;;
    
    "gtsam")
        echo -e "${YELLOW}安装GTSAM库...${NC}"
        if [ -f "tools/install_gtsam.sh" ]; then
            echo -e "${CYAN}运行GTSAM安装脚本...${NC}"
            bash tools/install_gtsam.sh
            if [ $? -eq 0 ]; then
                echo -e "${GREEN}✅ GTSAM安装完成${NC}"
                echo -e "${BLUE}现在可以编译pgo和hba包了：$0 build${NC}"
            else
                echo -e "${RED}❌ GTSAM安装失败${NC}"
            fi
        else
            echo -e "${RED}❌ 未找到GTSAM安装脚本${NC}"
        fi
        ;;
    
    "deps")
        echo -e "${CYAN}检查依赖包状态${NC}"
        echo "=========================="
        
        echo -e "${YELLOW}核心包状态:${NC}"
        
        # 检查GTSAM
        if [ -f "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
            # 从CMake配置文件中获取版本信息
            GTSAM_VERSION=$(grep "set(GTSAM_VERSION" /usr/local/lib/cmake/GTSAM/GTSAMConfigVersion.cmake 2>/dev/null | grep -o '[0-9.]*' | head -1)
            if [ -z "$GTSAM_VERSION" ]; then
                GTSAM_VERSION="4.1.1"  # 默认版本
            fi
            echo -e "  GTSAM: ${GREEN}✅ 已安装 (v$GTSAM_VERSION)${NC}"
        else
            echo -e "  GTSAM: ${RED}❌ 未安装${NC} - 运行 '$0 gtsam' 安装"
        fi
        
        # 检查包编译状态
        echo -e "${YELLOW}ROS2包状态:${NC}"
        packages=("interface" "livox_ros_driver2" "fastlio2" "pgo" "hba" "localizer")
        
        cd ws_livox 2>/dev/null || {
            echo -e "${RED}❌ 无法进入ws_livox目录${NC}"
            return 1
        }
        
        for pkg in "${packages[@]}"; do
            if [ -d "install/$pkg" ]; then
                echo -e "  $pkg: ${GREEN}✅ 已编译${NC}"
            elif [ -d "src/$pkg" ]; then
                echo -e "  $pkg: ${YELLOW}⏳ 未编译${NC}"
            else
                echo -e "  $pkg: ${RED}❌ 不存在${NC}"
            fi
        done
        
        echo ""
        echo -e "${CYAN}依赖要求:${NC}"
        echo "  pgo包: 需要GTSAM库 (位姿图优化)"
        echo "  hba包: 需要GTSAM库 (分层束调整)"  
        echo "  localizer包: 仅需PCL (ICP定位)"
        
        if [ ! -f "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
            echo ""
            echo -e "${YELLOW}推荐操作:${NC}"
            echo "  1. 安装GTSAM: $0 gtsam"
            echo "  2. 重新编译: $0 build"
        fi
        ;;
    
    "fix")
        echo -e "${YELLOW}修复已知编译问题...${NC}"
        cd ws_livox
        
        # 检查并修复livox_ros_driver2的ROS版本检测问题
        CMAKE_FILE="src/livox_ros_driver2/CMakeLists.txt"
        if [ -f "$CMAKE_FILE" ]; then
            echo "检查 livox_ros_driver2 CMakeLists.txt..."
            
            # 检查是否存在错误的HUMBLE_ROS检查
            if grep -q "if(HUMBLE_ROS STREQUAL \"humble\")" "$CMAKE_FILE"; then
                echo -e "${YELLOW}发现ROS版本检测问题，正在修复...${NC}"
                
                # 备份原文件
                cp "$CMAKE_FILE" "${CMAKE_FILE}.backup"
                
                # 修复版本检测
                sed -i 's/if(HUMBLE_ROS STREQUAL "humble")/if($ENV{ROS_DISTRO} STREQUAL "humble")/g' "$CMAKE_FILE"
                
                echo -e "${GREEN}✓ ROS版本检测已修复${NC}"
                echo -e "${BLUE}  备份文件: ${CMAKE_FILE}.backup${NC}"
            else
                echo -e "${GREEN}✓ ROS版本检测正常${NC}"
            fi
        else
            echo -e "${RED}❌ 未找到 livox_ros_driver2/CMakeLists.txt${NC}"
        fi
        
        # 检查其他潜在问题
        echo ""
        echo -e "${CYAN}系统环境检查:${NC}"
        echo "ROS版本: $ROS_DISTRO"
        echo "工作目录: $(pwd)"
        
        if [ -d "src" ]; then
            echo -e "源码目录: ${GREEN}✓${NC}"
        else
            echo -e "源码目录: ${RED}✗${NC}"
        fi
        
        echo ""
        echo -e "${GREEN}修复完成！现在可以运行 '$0 build' 重新编译${NC}"
        ;;
    
    "help"|"--help"|"-h"|"")
        show_help
        ;;
    
    *)
        echo -e "${RED}错误: 未知命令 '$1'${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac