#!/usr/bin/env bash
set -Eeuo pipefail

# FASTLIO2_ROS2 SLAM工具集合脚本（重构版）
# 目标：
#  - 消除硬编码路径，自动定位仓库根目录与工作区
#  - 统一日志输出与错误处理
#  - 提升健壮性（环境探测、依赖检查、优雅退出）

# ------------------------- 基础与通用 -------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 颜色与日志
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'
log() { printf "%b\n" "$*"; }
info() { log "${BLUE}$*${NC}"; }
ok()   { log "${GREEN}$*${NC}"; }
warn() { log "${YELLOW}$*${NC}"; }
err()  { log "${RED}$*${NC}"; }
die()  { err "$*"; exit 1; }

# 自动定位仓库根目录（优先 git，再回退到脚本上级）
ROOT_DIR="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "${ROOT_DIR}" ]]; then
  ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

# 工作区路径
WS_DIR="$ROOT_DIR/ws_livox"

# 进入仓库根目录
cd "$ROOT_DIR" 2>/dev/null || die "错误: 无法进入SLAM仓库目录: $ROOT_DIR"

# 安全 source（避免 set -u 触发未绑定变量错误）
source_safe() {
  # $1: 文件路径
  set +u
  # shellcheck disable=SC1090
  source "$1" 2>/dev/null || true
  set -u
}

# ROS环境加载
load_ros_env() {
  source_safe "/opt/ros/humble/setup.bash"
  if [[ -f "$WS_DIR/install/setup.bash" ]]; then
    source_safe "$WS_DIR/install/setup.bash"
  else
    warn "未找到工作区安装环境: $WS_DIR/install/setup.bash (首次使用请先编译)"
  fi
  command -v ros2 >/dev/null 2>&1 || die "未检测到 ros2，请确认已安装并正确 source ROS 环境"
}

show_help() {
    echo "========================================="
    echo "  MID360 FASTLIO2_ROS2 SLAM工具集"
    echo "========================================="
    echo ""
    echo -e "${YELLOW}使用方法:${NC}"
    echo "  $0 [命令]"
    echo ""
    echo -e "${YELLOW}可用命令:${NC}"
    echo -e "  ${CYAN}start${NC}     - 启动SLAM系统；支持子命令:"
    echo -e "                 start realtime         # 应用实机预设并启动"
    echo -e "                 start nofilter         # 实机启动且禁用过滤桥（LIO直接吃 /livox/lidar）"
    echo -e "                 start replay --bag <dir> [--rate 1.0] [--loop] [--no-dynamic-filter]  # 应用回放预设并启动；可临时关闭动态过滤"
    echo -e "  ${CYAN}start-basic${NC} - 仅启动基础SLAM (fastlio2)"
    echo -e "  ${CYAN}stop${NC}      - 停止SLAM系统"
    echo -e "  ${CYAN}status${NC}    - 检查系统状态"
    echo -e "  ${CYAN}restart${NC}   - 重启SLAM系统"
    echo -e "  ${CYAN}monitor${NC}   - 实时监控数据流"
    echo -e "  ${CYAN}optimize${NC} - 手动触发优化（PGO优先，自动回写LIO）"
    echo -e "  ${CYAN}topics${NC}    - 显示所有话题"
    echo -e "  ${CYAN}nodes${NC}     - 显示所有节点"
    echo -e "  ${CYAN}build${NC}     - 编译所有SLAM组件（完整系统）"
    echo -e "  ${CYAN}clean${NC}     - 清理编译文件"
    echo -e "  ${CYAN}fix${NC}       - 修复已知编译问题"
    echo -e "  ${CYAN}config${NC}    - 配置管理 (generate|validate|status|help)"
    echo -e "  ${CYAN}network${NC}   - 检查网络配置"
    echo -e "  ${CYAN}log${NC}       - 查看系统日志"
    echo -e "  ${CYAN}rviz${NC}      - 启动RViz可视化"
    echo -e "  ${CYAN}save${NC}      - 保存LIO/PGO/HBA数据（地图与轨迹）"
    echo -e "  ${CYAN}reconstruct${NC} - 使用patches+poses离线重建整图"
    echo -e "  ${CYAN}preset${NC}    - 应用配置预设 (realtime|replay) 并生成配置"
    echo -e "  ${CYAN}view${NC}      - 查看已保存的地图文件 (支持--rviz参数)"
    echo -e "  ${CYAN}maps${NC}      - 管理已保存的地图"
    echo -e "  ${CYAN}gridmap${NC}   - 将PCD地图转换为栅格地图"
    echo -e "  ${CYAN}refine${NC}    - 离线地图精细化处理（增强配准、去噪、结构保护）"
    echo -e "               支持去噪模式: --denoising-mode conservative|balanced|aggressive"
    echo -e "  ${CYAN}benchmark${NC} - 地图精细化工具性能基准测试（多线程优化效果）"
    echo -e "  ${CYAN}filter${NC}    - 动态对象过滤器功能 (test|viz|tune|status|enable|disable)"
    echo -e "               测试动态过滤、可视化调试、参数调优、状态检查、启用/禁用过滤"
    echo -e "  ${CYAN}record${NC}    - 开始录制SLAM数据"
    echo -e "  ${CYAN}record-stop${NC} - 停止当前录制"
    echo -e "  ${CYAN}validate${NC}  - 使用录制数据验证SLAM改进效果"
    # 已废弃：replay / replay-bag（改用 start replay）
    echo -e "  ${CYAN}bags${NC}      - 管理录制的数据包"
    echo -e "  ${CYAN}gtsam${NC}     - 安装GTSAM库（用于pgo和hba）"
    echo -e "  ${CYAN}deps${NC}      - 检查依赖包状态"
    echo -e "  ${CYAN}test${NC}      - 运行系统测试 (compile|unit|integration|filter|all)"
    echo -e "  ${CYAN}help${NC}      - 显示此帮助信息"
    echo ""
    echo -e "${YELLOW}示例:${NC}"
    echo "  $0 fix          # 修复编译问题"
    echo "  $0 build        # 重新编译系统"
    echo "  $0 start        # 启动SLAM系统"
    echo "  $0 status       # 检查运行状态"
    echo "  $0 filter test  # 测试动态过滤器"
    echo "  $0 filter viz   # 启动过滤器可视化"
    echo "  $0 test filter  # 运行过滤器测试"
}

# 额外启动参数传递给协同launch（避免 set -u 未绑定）
START_EXTRA_LAUNCH_ARGS=""

# ------------------------- 功能模块 -------------------------

# 等待优化完成（优先通过PGO服务查询）
wait_optimization_with_pgo() {
    local timeout_sec=${1:-60}
    local start_ts=$(date +%s)
    load_ros_env

    # 如果PGO优化查询服务可用，轮询其结果（更可靠）
    if ros2 service list 2>/dev/null | grep -q "/pgo/get_optimized_pose"; then
        echo -e "${CYAN}通过 pgo/get_optimized_pose 轮询优化结果...${NC}"
        while true; do
            if timeout 3s ros2 service call /pgo/get_optimized_pose interface/srv/GetOptimizedPose "{}" 2>/dev/null | grep -q "success: true"; then
                echo -e "${GREEN}PGO已返回优化结果${NC}"
                break
            fi
            now=$(date +%s)
            if [ $((now-start_ts)) -ge $timeout_sec ]; then
                echo -e "${YELLOW}等待超时(${timeout_sec}s)，继续后续流程${NC}"
                break
            fi
            echo -e "${BLUE}优化进行中...${NC}"
            sleep 2
        done
        return 0
    fi

    # 退化：沿用早期话题监测逻辑
    echo -e "${CYAN}PGO服务不可用，使用话题监测回退策略...${NC}"
    local i=0
    while [ $i -lt $timeout_sec ]; do
        if timeout 2s ros2 topic echo /slam/coordination_metrics --once 2>/dev/null | grep -q "optimization_active.*false" 2>/dev/null; then
            echo -e "${GREEN}优化完成（协调器指标）${NC}"
            break
        fi
        if timeout 2s ros2 topic echo /fastlio2/cooperation_status --once 2>/dev/null | grep -q "data.*0.0" 2>/dev/null; then
            echo -e "${GREEN}优化完成（FastLIO2状态）${NC}"
            break
        fi
        if [ $i -gt 30 ]; then
            echo -e "${BLUE}继续等待优化完成... (${i}/${timeout_sec}s)${NC}"
            sleep 3
            i=$((i+3))
        else
            echo -e "${BLUE}优化进行中... (${i}/${timeout_sec}s)${NC}"
            sleep 1
            i=$((i+1))
        fi
    done
}

# 启动协同SLAM系统
start_cooperative_slam_system() {
    info "=== 启动协同SLAM系统 ==="
    
    # 检查依赖
    if ! check_slam_dependencies; then
        err "❌ 依赖检查失败，无法启动完整系统"
        warn "尝试运行：$0 deps 检查缺失的依赖"
        return 1
    fi
    
    load_ros_env
    
    # 使用协同launch文件启动所有组件
    info "启动协同SLAM系统（真正的协同工作）..."
    warn "包含组件：Livox驱动 + FastLIO2 + PGO + HBA + Localizer + 优化协调器"
    warn "协同功能：双向优化反馈，智能协调器调度"

    # 传递可选的额外启动参数（例如 enable_filter_bridge:=false）
    ros2 launch fastlio2 cooperative_slam_system.launch.py ${START_EXTRA_LAUNCH_ARGS} &
    SLAM_SYSTEM_PID=$!
    echo $SLAM_SYSTEM_PID > /tmp/slam_coop.pid
    
    info "等待系统启动..."
    sleep 8
    
    # 检查核心组件启动状态
    local success=0
    local total=0
    
    # 检查各个节点
    components=("lio_node:FastLIO2" "pgo_node:PGO" "hba_node:HBA" "localizer_node:Localizer" "optimization_coordinator:协调器" "rviz2:可视化")
    
    for component in "${components[@]}"; do
        node_name="${component%%:*}"
        display_name="${component##*:}"
        total=$((total + 1))
        
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            ok "✅ $display_name 启动成功"
            success=$((success + 1))
        else
            err "❌ $display_name 启动失败"
        fi
    done
    
    echo ""
    if [ $success -eq $total ]; then
        ok "🎉 完整SLAM系统启动成功！($success/$total)"
        info "📊 主监控窗口：PGO RViz（显示实时建图+回环检测+轨迹优化）"
    elif [ $success -gt 0 ]; then
        warn "⚠️  部分组件启动成功 ($success/$total)"
        info "📊 主监控窗口：PGO RViz"
    else
        err "❌ 系统启动失败"
        return 1
    fi
    
    echo ""
    info "管理命令:"
    info "  状态检查: $0 status"
    info "  实时监控: $0 monitor"
    info "  保存地图: $0 save"
}

# 停止所有SLAM组件
stop_all_slam_components() {
    warn "停止所有SLAM组件..."
    
    # 1) 如果保存了launch的PID，优先干净关闭它及其子进程
    if [ -f /tmp/slam_coop.pid ]; then
        PID=$(cat /tmp/slam_coop.pid 2>/dev/null || echo "")
        if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
            warn "停止协同launch (PID: $PID)..."
            # 先尝试优雅终止
            kill "$PID" 2>/dev/null || true
            # 清理其子进程
            pkill -P "$PID" 2>/dev/null || true
            sleep 1
            # 如仍存活则强杀
            if kill -0 "$PID" 2>/dev/null; then
                kill -9 "$PID" 2>/dev/null || true
                pkill -9 -P "$PID" 2>/dev/null || true
            fi
        fi
        rm -f /tmp/slam_coop.pid
    fi

    # 2) 逐类强制关闭所有可能的独立进程（多重保险）
    patterns=(
        "livox_ros_driver2_node"
        "point_cloud_filter_bridge_node"
        "lio_node"
        "pgo_node"
        "hba_node"
        "localizer_node"
        "optimization_coordinator"
        "robot_state_publisher"
        "static_transform_publisher"
        "rviz2"
        "ros2 bag play"
    )
    for pat in "${patterns[@]}"; do
        pkill -f "$pat" 2>/dev/null || true
    done

    # 3) 兜底：杀掉残留的launch进程
    pkill -f "ros2 launch.*fastlio2" 2>/dev/null || true
    pkill -f "ros2 launch.*pgo" 2>/dev/null || true
    pkill -f "ros2 launch.*hba" 2>/dev/null || true
    pkill -f "ros2 launch.*localizer" 2>/dev/null || true

    # 4) 等待节点消失（最多3秒）
    for i in 1 2 3; do
        if ! ros2 node list 2>/dev/null | grep -E "(livox|lio_node|pgo_node|hba_node|localizer_node|optimization_coordinator|point_cloud_filter_bridge|robot_state_publisher|static_transform)" >/dev/null; then
            break
        fi
        sleep 1
    done

    ok "✅ 所有SLAM组件已停止（如仍有残留，请再执行一次 stop）"
}

# 检查完整SLAM状态
check_full_slam_status() {
    info "完整SLAM系统状态检查"
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
                ok "  $description: ✅ 运行中"
                active_count=$((active_count + 1))
            else
                err "  $description: ❌ 未运行"
            fi
        else
            warn "  $description: ⏸  未编译"
        fi
    done
    
    echo ""
    warn "系统状态摘要:"
    echo "  活跃组件: $active_count/$total_expected"
    
    if [ $active_count -eq $total_expected ] && [ $total_expected -gt 0 ]; then
        ok "  整体状态: ✅ 系统正常运行"
    elif [ $active_count -gt 0 ]; then
        warn "  整体状态: ⚠️  部分组件运行"
    else
        err "  整体状态: ❌ 系统未运行"
    fi
    
    # 检查话题活跃度
    echo ""
    warn "关键话题状态:"
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
            ok "  $description: ✅ 数据流正常"
        else
            err "  $description: ❌ 无数据流"
        fi
    done

    # 检查协同系统（如果存在）
    if ros2 node list 2>/dev/null | grep -qE "(pgo|hba|localizer|optimization)"; then
        echo ""
        warn "协同系统状态:"

        # 协同服务检查
        coop_services=$(ros2 service list 2>/dev/null | grep -E "(update_pose|sync_state|trigger_optimization)" | wc -l)
        if [ $coop_services -gt 0 ]; then
            ok "  协同服务: ✅ $coop_services 个服务可用"
        else
            warn "  协同服务: ⚠️  无协同服务"
        fi

        # 协同话题检查
        coop_topics=(
            "/fastlio2/cooperation_status:协同状态"
            "/slam/coordination_metrics:协调指标"
            "/fastlio2/performance_metrics:性能指标"
            "/localizer/filter_stats:动态过滤统计"
            "/localizer/filtered_cloud:过滤后点云"
            "/localizer/dynamic_cloud:动态点云"
        )

        for topic_info in "${coop_topics[@]}"; do
            topic="${topic_info%%:*}"
            description="${topic_info##*:}"

            if timeout 2s ros2 topic hz "$topic" --once 2>/dev/null | grep -q "Hz"; then
                ok "  $description: ✅ 活跃"
            else
                warn "  $description: ⚠️  无数据"
            fi
        done
    fi
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

# 解析主命令（在 set -u 下需安全取参）
CMD="${1-}"
if [[ -z "${CMD}" ]]; then
  show_help
  exit 0
fi

case "${CMD}" in
    "start")
        # 子命令下沉到函数，保持入口清晰
        start_realtime() {
            echo -e "${CYAN}应用实机预设并启动SLAM系统...${NC}"
            python3 tools/apply_preset.py --mode realtime || die "应用实机预设失败"
            start_cooperative_slam_system
        }

        start_nofilter() {
            echo -e "${CYAN}应用实机预设并禁用过滤桥，直接使用原始点云...${NC}"
            python3 tools/apply_preset.py --mode realtime || die "应用实机预设失败"
            START_EXTRA_LAUNCH_ARGS="enable_filter_bridge:=false"
            start_cooperative_slam_system
        }

        patch_cfg_disable_dyn_filter() {
            py_edit_cfg='import sys,yaml,shutil
from pathlib import Path
root=Path(".")
src_loc=root/"ws_livox/src/localizer/config/localizer.yaml"
inst_loc=root/"install/localizer/share/localizer/config/localizer.yaml"
src_pgo=root/"ws_livox/src/pgo/config/pgo.yaml"
inst_pgo=root/"install/pgo/share/pgo/config/pgo.yaml"
bak1=Path("/tmp/localizer.yaml.bak"); bak2=Path("/tmp/localizer.install.yaml.bak")
bak3=Path("/tmp/pgo.yaml.bak"); bak4=Path("/tmp/pgo.install.yaml.bak")
def patch_loc(p:Path):
    if not p.exists(): return
    data=yaml.safe_load(p.read_text())
    d=data.get("dynamic_filter",{})
    d["enable"]=False
    data["dynamic_filter"]=d
    p.write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
def patch_pgo(p:Path):
    if not p.exists(): return
    data=yaml.safe_load(p.read_text())
    data["cloud_topic"]="/slam/body_cloud"
    p.write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
shutil.copy2(src_loc,bak1) if src_loc.exists() else None
shutil.copy2(inst_loc,bak2) if inst_loc.exists() else None
shutil.copy2(src_pgo,bak3) if src_pgo.exists() else None
shutil.copy2(inst_pgo,bak4) if inst_pgo.exists() else None
patch_loc(src_loc); patch_loc(inst_loc)
patch_pgo(src_pgo); patch_pgo(inst_pgo)
print("OK")
'
            python3 -c "$py_edit_cfg" >/dev/null 2>&1 || true
        }

        restore_cfg_if_any() {
            for pair in \
                "/tmp/localizer.yaml.bak:ws_livox/src/localizer/config/localizer.yaml" \
                "/tmp/localizer.install.yaml.bak:install/localizer/share/localizer/config/localizer.yaml" \
                "/tmp/pgo.yaml.bak:ws_livox/src/pgo/config/pgo.yaml" \
                "/tmp/pgo.install.yaml.bak:install/pgo/share/pgo/config/pgo.yaml"; do
                IFS=":" read -r bak dst <<< "$pair"
                if [[ -f "$bak" ]]; then
                    cp -f "$bak" "$dst" 2>/dev/null || true
                    rm -f "$bak" 2>/dev/null || true
                fi
            done
        }

        start_replay() {
            shift 2 || true
            local BAG=""
            local RATE="1.0"
            local LOOP="false"
            local TOPICS=""
            local REMAPS=()
            local START_OFFSET=""
            local NO_DYN_FILTER="false"
            while [[ $# -gt 0 ]]; do
                case "$1" in
                    --bag)          BAG="${2-}"; shift 2;;
                    --rate)         RATE="${2-}"; shift 2;;
                    --loop)         LOOP="true"; shift;;
                    --topics)       TOPICS="${2-}"; shift 2;;
                    --remap)        REMAPS+=("${2-}"); shift 2;;
                    --start-offset) START_OFFSET="${2-}"; shift 2;;
                    --no-dynamic-filter|--disable-dynamic-filter)
                                      NO_DYN_FILTER="true"; shift;;
                    *)              echo -e "${YELLOW}忽略未知参数:${NC} $1"; shift;;
                esac
            done

            echo -e "${CYAN}应用回放预设并启动SLAM系统...${NC}"
            python3 tools/apply_preset.py --mode replay || die "应用回放预设失败"
            load_ros_env
            info "=== 启动协同SLAM（回放模式） ==="

            local cmd=(ros2 launch fastlio2 cooperative_slam_system.launch.py use_bag:=true use_sim_time:=true)
            local AUTO_PLAY="true"
            if [[ -n "$TOPICS" || ${#REMAPS[@]} -gt 0 ]]; then AUTO_PLAY="false"; fi

            if [[ "$NO_DYN_FILTER" == "true" ]]; then
                echo -e "${YELLOW}临时禁用 Localizer 动态过滤，并将 PGO 订阅改为 /slam/body_cloud${NC}"
                patch_cfg_disable_dyn_filter
            fi

            if [[ -n "$BAG" && "$AUTO_PLAY" == "true" ]]; then
                cmd+=(auto_play_bag:=true bag_path:="$BAG" bag_rate:="$RATE")
                if [[ "$LOOP" == "true" ]]; then cmd+=(bag_loop:=true); else cmd+=(bag_loop:=false); fi
            else
                cmd+=(auto_play_bag:=false)
            fi

            "${cmd[@]}" &
            local SLAM_SYSTEM_PID=$!
            echo $SLAM_SYSTEM_PID > /tmp/slam_coop.pid
            ok "🎬 协同SLAM已启动（回放模式），PID=$SLAM_SYSTEM_PID"

            cleanup() {
                if [[ -f /tmp/slam_bag.pid ]]; then
                    local BAGPID=$(cat /tmp/slam_bag.pid 2>/dev/null || echo "");
                    kill "$BAGPID" 2>/dev/null || true
                    rm -f /tmp/slam_bag.pid
                fi
                if [[ -f /tmp/slam_coop.pid ]]; then
                    local LPID=$(cat /tmp/slam_coop.pid 2>/dev/null || echo "");
                    kill "$LPID" 2>/dev/null || true
                    rm -f /tmp/slam_coop.pid
                fi
            }
            trap 'cleanup; restore_cfg_if_any 2>/dev/null || true' INT TERM

            if [[ -n "$BAG" && "$AUTO_PLAY" == "false" ]]; then
                local bag_cmd=(ros2 bag play "$BAG" --rate "$RATE" --clock)
                if [[ "$LOOP" == "true" ]]; then bag_cmd+=(--loop); fi
                if [[ -n "$START_OFFSET" ]]; then bag_cmd+=(--start-offset "$START_OFFSET"); fi
                if [[ -n "$TOPICS" ]]; then
                    bag_cmd+=(--topics)
                    # shellcheck disable=SC2206
                    local TOPIC_ARR=($TOPICS)
                    for t in "${TOPIC_ARR[@]}"; do bag_cmd+=("$t"); done
                fi
                local LIO_CFG="ws_livox/src/fastlio2/config/lio.yaml"
                if [[ -f "$LIO_CFG" ]] && grep -q "lidar_topic: /livox/lidar_filtered" "$LIO_CFG"; then
                    REMAPS+=("/livox/lidar:=/livox/lidar_filtered")
                    warn "检测到 LIO 订阅 /livox/lidar_filtered，已自动添加 remap: /livox/lidar:=/livox/lidar_filtered"
                fi
                for r in "${REMAPS[@]}"; do bag_cmd+=(--remap "$r"); done
                if [[ -z "$TOPICS" ]]; then
                    bag_cmd+=(--topics /livox/imu /livox/lidar)
                    warn "未指定 --topics，默认只回放: /livox/imu /livox/lidar"
                fi
                info "自动回放: ${bag_cmd[*]}"
                "${bag_cmd[@]}" &
                echo $! > /tmp/slam_bag.pid
            else
                warn "使用 'ros2 bag play <dir>' 可手动回放；或通过 --bag 自动回放"
            fi

            echo -e "${YELLOW}按 Ctrl+C 停止回放并关闭SLAM${NC}"
            wait "$SLAM_SYSTEM_PID" 2>/dev/null || true
            cleanup; restore_cfg_if_any 2>/dev/null || true
        }

        SUBCMD="${2-}"
        case "$SUBCMD" in
            "realtime"|"") start_realtime ;;
            "nofilter")     start_nofilter ;;
            "replay")       start_replay "$@" ;;
            *)
                echo -e "${YELLOW}未知的start子命令:${NC} $SUBCMD"
                echo "用法: $0 start [realtime|nofilter|replay ...]"
                exit 1
                ;;
        esac
        ;;

    "replay"|"replay-bag")
        echo -e "${RED}该命令已废弃:${NC} 请使用:"
        echo "  $0 start replay --bag <dir> [--rate 1.0] [--loop] [--topics "\"/imu /lidar\""] [--remap from:=to]"
        exit 1
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
        start_cooperative_slam_system
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


    "optimize")
        echo -e "${CYAN}手动触发协同优化（PGO→回写LIO→同步轨迹）...${NC}"
        load_ros_env
        echo ""

        echo -e "${YELLOW}触发PGO优化...${NC}"
        ros2 service call /coordinator/trigger_optimization interface/srv/TriggerOptimization "{optimization_level: 1, include_pgo: true, include_hba: false, include_localizer: false, drift_threshold: 0.2, emergency_mode: false}" --timeout 10 2>/dev/null || true

        echo -e "${CYAN}等待优化完成...${NC}"
        wait_optimization_with_pgo 60
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
        
        # 检查GTSAM依赖（可选，仅影响PGO/HBA）
        HAS_GTSAM=0
        if [ -f "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake" ]; then
            HAS_GTSAM=1
            echo -e "${GREEN}✓ 检测到GTSAM，启用PGO/HBA编译${NC}"
        else
            echo -e "${YELLOW}⚠️  未检测到GTSAM，将跳过PGO/HBA编译${NC}"
            echo -e "${YELLOW}   如需编译PGO/HBA，请先运行: $0 gtsam${NC}"
        fi
        
        # 按依赖顺序编译所有包
        echo "正在编译所有SLAM组件..."
        
        # 核心包（必需）- 按依赖顺序排列
        core_packages=("interface" "livox_ros_driver2" "localizer" "point_cloud_filter" "fastlio2" "cooperation")
        # 可选包（需要GTSAM）
        extended_packages=()
        if [ $HAS_GTSAM -eq 1 ]; then
            extended_packages=("pgo" "hba")
        fi

        # 检查动态过滤器是否已集成
        if [ -f "src/localizer/src/localizers/dynamic_object_filter.cpp" ]; then
            echo -e "${GREEN}✓ 动态对象过滤器已集成${NC}"
        else
            echo -e "${YELLOW}⚠️  动态对象过滤器未找到${NC}"
        fi
        
        # 编译核心包
        for i in "${!core_packages[@]}"; do
            pkg="${core_packages[$i]}"
            step=$((i + 1))
            echo -e "${CYAN}[${step}/6] 编译 ${pkg}...${NC}"
            colcon build --symlink-install --packages-select "${pkg}" --cmake-args -DCMAKE_BUILD_TYPE=Release
            if [ $? -ne 0 ]; then
                echo -e "${RED}❌ ${pkg}编译失败${NC}"
                echo -e "${YELLOW}提示：可能存在依赖问题，请检查错误信息${NC}"
                exit 1
            fi
            # 每次编译后立即source环境，确保后续包能找到依赖
            source_safe install/setup.bash
            echo -e "${GREEN}✓ ${pkg}编译成功${NC}"
        done
        
        # 编译扩展包
        if [ ${#extended_packages[@]} -gt 0 ]; then
            for i in "${!extended_packages[@]}"; do
                pkg="${extended_packages[$i]}"
                step=$((i + 7))  # 继续计数（前面6个核心包）
                echo -e "${CYAN}[${step}/8] 编译 ${pkg}...${NC}"
                colcon build --symlink-install --packages-select "${pkg}" --cmake-args -DCMAKE_BUILD_TYPE=Release
                if [ $? -ne 0 ]; then
                    echo -e "${YELLOW}⚠️  ${pkg}编译失败，但不影响基本功能${NC}"
                else
                    echo -e "${GREEN}✓ ${pkg}编译成功${NC}"
                fi
            done
        else
            echo -e "${YELLOW}⏭  跳过PGO/HBA编译（GTSAM未安装）${NC}"
        fi
        
        echo -e "${GREEN}🎉 完整SLAM系统编译完成！${NC}"
        echo ""
        echo -e "${YELLOW}完整编译结果验证:${NC}"
        
        # 验证所有包
        all_packages=("interface" "livox_ros_driver2" "point_cloud_filter" "fastlio2" "localizer" "cooperation" "pgo" "hba")
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
        SUBCMD="${2-}"
        case "$SUBCMD" in
            "generate")
                echo -e "${GREEN}生成统一配置文件...${NC}"
                echo "================================="

                # 检查配置生成器是否存在
                if [ ! -f "tools/config_generator.py" ]; then
                    echo -e "${RED}配置生成器不存在: tools/config_generator.py${NC}"
                    exit 1
                fi

                # 检查主配置文件是否存在
                if [ ! -f "config/master_config.yaml" ]; then
                    echo -e "${RED}主配置文件不存在: config/master_config.yaml${NC}"
                    echo "请先创建主配置文件"
                    exit 1
                fi

                # 运行配置生成器
                python3 tools/config_generator.py --master-config config/master_config.yaml

                if [ $? -eq 0 ]; then
                    echo -e "${GREEN}✓ 配置文件生成成功！${NC}"
                    echo -e "${YELLOW}注意: 外参配置已统一，应该能解决轨迹飘移问题${NC}"
                else
                    echo -e "${RED}✗ 配置文件生成失败${NC}"
                    exit 1
                fi
                ;;

            "validate")
                echo -e "${CYAN}验证配置一致性...${NC}"
                echo "================================="

                if [ ! -f "tools/config_generator.py" ]; then
                    echo -e "${RED}配置生成器不存在: tools/config_generator.py${NC}"
                    exit 1
                fi

                python3 tools/config_generator.py --master-config config/master_config.yaml --validate-only
                ;;

            "status")
                echo -e "${CYAN}配置状态检查:${NC}"
                echo "================================="

                # 检查主配置文件
                if [ -f "config/master_config.yaml" ]; then
                    echo -e "${GREEN}✓ 主配置文件存在${NC}"
                else
                    echo -e "${RED}✗ 主配置文件缺失${NC}"
                fi

                # 检查组件配置文件
                CONFIG_FILES=(
                    "ws_livox/src/fastlio2/config/lio.yaml:FAST-LIO2配置"
                    "ws_livox/src/livox_ros_driver2/config/MID360_config.json:Livox驱动配置"
                    "ws_livox/src/point_cloud_filter/config/point_cloud_filter.yaml:过滤桥参数"
                    "ws_livox/src/localizer/config/localizer.yaml:Localizer配置"
                    "ws_livox/src/pgo/config/pgo.yaml:PGO配置"
                    "ws_livox/src/hba/config/hba.yaml:HBA配置"
                    "config/launch_config.yaml:启动配置"
                )

                for config_info in "${CONFIG_FILES[@]}"; do
                    IFS=':' read -r file_path description <<< "$config_info"
                    if [ -f "$file_path" ]; then
                        echo -e "${GREEN}✓ $description存在${NC}"

                        # 检查是否由配置生成器生成
                        if grep -q "由配置生成器自动生成" "$file_path" 2>/dev/null; then
                            echo -e "  ${CYAN}  └─ 由配置生成器生成${NC}"
                        else
                            echo -e "  ${YELLOW}  └─ 手动配置文件${NC}"
                        fi
                    else
                        echo -e "${RED}✗ $description缺失${NC}"
                    fi
                done

                # 显示当前配置摘要
                echo ""
                echo "当前配置摘要:"
                echo "工作目录: $(pwd)"
                echo "ROS版本: $(ros2 --version 2>/dev/null || echo '未加载')"

                FASTLIO_CONFIG="ws_livox/src/fastlio2/config/lio.yaml"
                if [ -f "$FASTLIO_CONFIG" ]; then
                    echo ""
                    echo "FAST-LIO2配置:"
                    echo "  IMU话题: $(grep 'imu_topic:' "$FASTLIO_CONFIG" | awk '{print $2}' || echo '未设置')"
                    echo "  点云话题: $(grep 'lidar_topic:' "$FASTLIO_CONFIG" | awk '{print $2}' || echo '未设置')"
                    echo "  扫描分辨率: $(grep 'scan_resolution:' "$FASTLIO_CONFIG" | awk '{print $2}' || echo '未设置')"
                    echo "  外参平移: $(grep 't_il:' "$FASTLIO_CONFIG" | cut -d':' -f2 || echo '未设置')"
                fi

                CONFIG_FILE="ws_livox/src/livox_ros_driver2/config/MID360_config.json"
                if [ -f "$CONFIG_FILE" ]; then
                    echo ""
                    echo "Livox驱动配置:"
                    echo "  主机IP: $(grep -o '"cmd_data_ip"[[:space:]]*:[[:space:]]*"[^"]*"' "$CONFIG_FILE" | grep -o '[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*' | head -1 || echo '未设置')"
                    echo "  雷达IP: $(grep -o '"ip"[[:space:]]*:[[:space:]]*"[^"]*"' "$CONFIG_FILE" | grep -o '[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*' | head -1 || echo '未设置')"
                fi
                ;;

            "help"|"")
                echo -e "${CYAN}配置管理帮助:${NC}"
                echo "================================="
                echo "用法: $0 config [子命令]"
                echo ""
                echo "可用子命令:"
                echo -e "  ${YELLOW}generate${NC}  - 从主配置文件生成所有组件配置"
                echo -e "  ${YELLOW}validate${NC}  - 验证配置文件一致性"
                echo -e "  ${YELLOW}status${NC}    - 检查配置文件状态"
                echo -e "  ${YELLOW}help${NC}      - 显示此帮助信息"
                echo ""
                echo "示例:"
                echo "  $0 config generate   # 生成配置文件"
                echo "  $0 config validate   # 验证配置"
                echo "  $0 config status     # 查看状态"
                ;;

            *)
                echo -e "${RED}未知的config子命令: ${SUBCMD}${NC}"
                echo "使用 '$0 config help' 查看可用命令"
                exit 1
                ;;
        esac
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

        # 优先使用增强版可视化配置，其次回退到安装目录的标准配置
        RVIZ_CFG_SRC="ws_livox/src/fastlio2/rviz/enhanced_fastlio2.rviz"
        RVIZ_CFG_INSTALL_ENH="ws_livox/install/fastlio2/share/fastlio2/rviz/enhanced_fastlio2.rviz"
        RVIZ_CFG_INSTALL_STD="ws_livox/install/fastlio2/share/fastlio2/rviz/fastlio2.rviz"

        if [ -f "$RVIZ_CFG_SRC" ]; then
            echo "使用增强版RViz配置(源): $RVIZ_CFG_SRC"
            rviz2 -d "$RVIZ_CFG_SRC" &
        elif [ -f "$RVIZ_CFG_INSTALL_ENH" ]; then
            echo "使用增强版RViz配置(安装): $RVIZ_CFG_INSTALL_ENH"
            rviz2 -d "$RVIZ_CFG_INSTALL_ENH" &
        elif [ -f "$RVIZ_CFG_INSTALL_STD" ]; then
            echo "使用标准RViz配置(安装): $RVIZ_CFG_INSTALL_STD"
            rviz2 -d "$RVIZ_CFG_INSTALL_STD" &
        else
            echo "未找到预置RViz配置，启动默认RViz..."
            rviz2 &
        fi

        echo "RViz已在后台启动"
        ;;

    
    "save")
        echo -e "${YELLOW}保存SLAM地图与轨迹 (LIO/PGO/HBA)...${NC}"
        
        # 检查ROS环境
        load_ros_env
        
        # 检查SLAM节点是否运行
        if ! ros2 node list | grep -qE "(lio_node|fastlio2_node)"; then
            echo -e "${RED}❌ FAST-LIO2节点未运行${NC}"
            echo "请先运行: $0 start"
            exit 1
        fi

        # 检查协同系统是否运行，如果是则触发优化保存
        if ros2 node list | grep -q "optimization_coordinator"; then
            echo -e "${CYAN}检测到协同SLAM系统，将保存优化后的地图...${NC}"

            # 触发一次全面优化以获取最佳地图质量
            echo -e "${YELLOW}触发最终优化...${NC}"
            ros2 service call /coordinator/trigger_optimization interface/srv/TriggerOptimization "{optimization_level: 2, include_pgo: true, include_hba: true, include_localizer: true, drift_threshold: 0.1, emergency_mode: false}" --timeout 30 2>/dev/null || true

            echo -e "${CYAN}等待优化完成...${NC}"
            wait_optimization_with_pgo 60
        fi
        
        # 确保saved_maps目录存在
        mkdir -p saved_maps
        
        # 使用一键保存脚本（优先）
        if [ -f "tools/save_maps_bundle.py" ]; then
            echo -e "${CYAN}调用一键保存工具 (LIO/PGO/HBA)...${NC}"
            # 若HBA节点在运行，则顺带触发一次细化并等待片刻再保存轨迹
            if ros2 node list 2>/dev/null | grep -q "hba_node"; then
                python3 tools/save_maps_bundle.py -o saved_maps --hba-refine --hba-wait-seconds 30
            else
                python3 tools/save_maps_bundle.py -o saved_maps
            fi
            rc=$?
        elif [ -f "tools/save_map_simple.py" ]; then
            echo -e "${YELLOW}回退到简化保存工具 (仅LIO)...${NC}"
            python3 tools/save_map_simple.py saved_maps
            rc=$?
        else
            echo -e "${RED}❌ 未找到保存工具${NC}"
            echo "请确保存在 tools/save_maps_bundle.py 或 tools/save_map_simple.py"
            exit 1
        fi

        if [ ${rc:-1} -eq 0 ]; then
            echo -e "${GREEN}✅ 保存完成${NC}"
            echo -e "${CYAN}输出目录内容:${NC}"
            ls -lt saved_maps | head -10 || true
            echo -e "${YELLOW}提示:${NC} 可用 '$0 view <map.pcd> --rviz' 在RViz中查看（配置已预置 /saved_map 显示）"
        else
            echo -e "${RED}❌ 保存失败${NC}"
            exit 1
        fi
        ;;

    "reconstruct")
        shift || true
        # 用法： slam_tools.sh reconstruct --pgo-dir DIR [--poses poses.txt] --output out.pcd [--voxel 0.05]
        if [ ! -f "tools/reconstruct_map_from_patches.py" ]; then
            echo -e "${RED}❌ 未找到重建脚本: tools/reconstruct_map_from_patches.py${NC}"
            exit 1
        fi
        if [[ $# -eq 0 ]]; then
            echo -e "${YELLOW}用法:${NC} $0 reconstruct --pgo-dir <dir> [--poses <poses.txt>] --output <out.pcd> [--downsample-voxel 0.05]"
            exit 0
        fi
        echo -e "${CYAN}离线重建整图...${NC}"
        python3 tools/reconstruct_map_from_patches.py "$@"
        ;;

    "preset")
        # 用法: ./tools/slam_tools.sh preset realtime|replay
        MODE="${2-}"
        if [[ -z "$MODE" ]]; then
            echo -e "${YELLOW}用法:${NC} $0 preset {realtime|replay}"
            exit 1
        fi
        if [[ "$MODE" != "realtime" && "$MODE" != "replay" ]]; then
            echo -e "${RED}无效模式:${NC} $MODE (应为 realtime 或 replay)"
            exit 1
        fi
        echo -e "${CYAN}应用预设: $MODE ...${NC}"
        python3 tools/apply_preset.py --mode "$MODE"
        rc=$?
        if [ $rc -eq 0 ]; then
            echo -e "${GREEN}✅ 预设已应用并生成配置完成${NC}"
            echo -e "${YELLOW}提示:${NC} 重新启动系统以生效: $0 restart"
        else
            echo -e "${RED}❌ 预设应用失败 (返回码 $rc)${NC}"
            exit $rc
        fi
        ;;
    
    "view")
        if [ -z "${2-}" ]; then
            echo -e "${YELLOW}查看可用的地图文件:${NC}"
            if [ -d "saved_maps" ]; then
                echo "saved_maps 目录中的文件:"
                ls -la saved_maps/ 2>/dev/null
                echo ""
                echo -e "${CYAN}使用方法:${NC}"
                echo "  $0 view <点云文件路径> [选项]"
                echo ""
                echo -e "${CYAN}选项:${NC}"
                echo "  --rviz          在RViz中显示"
                echo "  --topic <name>  自定义话题名称 (配合--rviz)"
                echo "  --frame <name>  自定义坐标系 (配合--rviz)"
                echo ""
                echo -e "${CYAN}示例:${NC}"
                echo "  $0 view saved_maps/latest.pcd"
                echo "  $0 view saved_maps/latest.pcd --rviz"
            else
                echo -e "${RED}❌ saved_maps目录不存在${NC}"
                echo "请先运行 '$0 save' 保存地图"
            fi
        else
            PCD_FILE="${2-}"
            if [ ! -f "$PCD_FILE" ]; then
                echo -e "${RED}❌ PCD文件不存在: $PCD_FILE${NC}"
                exit 1
            fi

            # 检查是否使用RViz显示
            if [[ "$*" == *"--rviz"* ]]; then
                echo -e "${BLUE}🗺️  在RViz中显示保存的地图${NC}"

                # 解析参数
                TOPIC_NAME="/saved_map"
                FRAME_ID="map"
                shift 2  # 跳过命令和文件名
                while [[ $# -gt 0 ]]; do
                    case $1 in
                        --topic)
                            TOPIC_NAME="$2"
                            shift 2
                            ;;
                        --frame)
                            FRAME_ID="$2"
                            shift 2
                            ;;
                        --rviz)
                            shift
                            ;;
                        *)
                            shift
                            ;;
                    esac
                done

                echo "文件: $PCD_FILE"
                echo "话题: $TOPIC_NAME"
                echo "坐标系: $FRAME_ID"

                # 启动地图发布节点
                load_ros_env
                echo -e "${GREEN}📡 启动地图发布节点...${NC}"
                python3 tools/publish_saved_map.py "$PCD_FILE" --topic "$TOPIC_NAME" --frame "$FRAME_ID" &
                MAP_PUB_PID=$!

                # 等待一秒让发布节点启动
                sleep 2

                # 启动RViz
                echo -e "${GREEN}🚀 启动RViz...${NC}"
                RVIZ_CONFIG="ws_livox/src/fastlio2/rviz/enhanced_fastlio2.rviz"
                if [ ! -f "$RVIZ_CONFIG" ]; then
                    RVIZ_CONFIG="ws_livox/install/fastlio2/share/fastlio2/rviz/fastlio2.rviz"
                fi
                rviz2 -d "$RVIZ_CONFIG" &
                RVIZ_PID=$!

                echo -e "${YELLOW}💡 RViz配置已预置 'Saved Map' 显示:${NC}"
                echo "   - 话题: $TOPIC_NAME（默认 /saved_map）"
                echo "   - Fixed Frame: $FRAME_ID（默认 map）"
                echo ""
                echo -e "${YELLOW}⚠️  按Ctrl+C停止显示${NC}"

                # 等待用户中断
                trap "echo ''; echo '🛑 停止地图显示...'; kill $MAP_PUB_PID 2>/dev/null; kill $RVIZ_PID 2>/dev/null; exit 0" SIGINT
                wait
            else
                echo -e "${BLUE}🗺️  使用Open3D显示保存的地图${NC}"
                if [ -f "tools/view_saved_map.py" ]; then
                    python3 tools/view_saved_map.py "$PCD_FILE"
                else
                    echo -e "${RED}❌ 地图查看工具不存在${NC}"
                    echo "请确保 tools/view_saved_map.py 文件存在"
                fi
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

    "record")
        echo -e "${YELLOW}🎥 开始录制SLAM数据...${NC}"

        # 确保已加载ROS环境与工作区（否则rosbag无法识别自定义消息类型）
        load_ros_env

        # 检查录制工具是否存在
        RECORD_TOOL="$SCRIPT_DIR/record_data.py"
        if [ ! -f "$RECORD_TOOL" ]; then
            echo -e "${RED}❌ 录制工具不存在: $RECORD_TOOL${NC}"
            exit 1
        fi

        # 启动录制，过滤掉命令名本身
        shift  # 移除第一个参数 "record"
        python3 "$RECORD_TOOL" "$@"
        ;;

    "record-stop")
        echo -e "${YELLOW}🛑 停止当前录制...${NC}"

        # 查找并停止rosbag录制进程
        ROSBAG_PIDS=$(pgrep -f "ros2 bag record")
        if [ -n "$ROSBAG_PIDS" ]; then
            echo -e "${BLUE}找到录制进程: $ROSBAG_PIDS${NC}"
            for pid in $ROSBAG_PIDS; do
                echo -e "${YELLOW}停止进程 $pid...${NC}"
                kill -SIGINT $pid 2>/dev/null || kill -TERM $pid 2>/dev/null || kill -KILL $pid 2>/dev/null
            done
            echo -e "${GREEN}✅ 录制已停止${NC}"
        else
            echo -e "${YELLOW}⚠️  未找到活动的录制进程${NC}"
        fi
        ;;

    "validate")
        echo -e "${YELLOW}🧪 启动SLAM改进效果验证...${NC}"
        if [ -f "$SCRIPT_DIR/replay_validation.py" ]; then
            shift  # 移除 'validate' 参数
            python3 "$SCRIPT_DIR/replay_validation.py" "$@"
        else
            echo -e "${RED}❌ 验证工具不存在: $SCRIPT_DIR/replay_validation.py${NC}"
            exit 1
        fi
        ;;

    "replay")
        echo -e "${YELLOW}▶️ 回放SLAM数据...${NC}"
        if [ -f "$SCRIPT_DIR/replay_validation.py" ]; then
            shift  # 移除 'replay' 参数

            # 默认使用仿真时间并限制回放速率，避免RViz/TF队列溢出
            # 如果未显式指定 --rate，则注入 --rate 1.0
            RATE_SPECIFIED=false
            RVIZ_REQUESTED=false
            ARGS=()
            for arg in "$@"; do
                if [[ "$arg" == "--rate" ]]; then
                    RATE_SPECIFIED=true
                fi
                if [[ "$arg" == "--rviz" ]]; then
                    RVIZ_REQUESTED=true
                    continue  # 该标志仅用于此脚本，不直接传入python
                fi
                ARGS+=("$arg")
            done

            if ! $RATE_SPECIFIED; then
                echo -e "${BLUE}ℹ 未指定 --rate，使用默认 1.0x 回放${NC}"
                ARGS+=("--rate" "1.0")
            fi

            if $RVIZ_REQUESTED; then
                # 启动时允许RViz，由python脚本负责将 use_sim_time 置为 true
                python3 "$SCRIPT_DIR/replay_validation.py" "${ARGS[@]}"
            else
                # 默认不启动RViz，专注离线验证，避免可视化队列压力
                python3 "$SCRIPT_DIR/replay_validation.py" --no-rviz "${ARGS[@]}"
            fi
        else
            echo -e "${RED}❌ 回放工具不存在: $SCRIPT_DIR/replay_validation.py${NC}"
            exit 1
        fi
        ;;

    "bags")
        echo -e "${CYAN}📦 管理录制数据包${NC}"
        if [ -f "$SCRIPT_DIR/replay_validation.py" ]; then
            python3 "$SCRIPT_DIR/replay_validation.py" --list-bags
        else
            echo -e "${RED}❌ 数据包管理工具不存在${NC}"
            exit 1
        fi
        ;;

    "gridmap")
        echo -e "${YELLOW}🗺️  PCD地图转栅格地图工具${NC}"

        # 检查转换工具是否存在
        if [ ! -f "$SCRIPT_DIR/pcd_to_gridmap.py" ]; then
            echo -e "${RED}❌ 转换工具不存在: $SCRIPT_DIR/pcd_to_gridmap.py${NC}"
            exit 1
        fi

        # 检查保存目录是否存在
        MAPS_DIR="$(dirname "$SCRIPT_DIR")/saved_maps"
        if [ ! -d "$MAPS_DIR" ]; then
            echo -e "${RED}❌ 地图保存目录不存在: $MAPS_DIR${NC}"
            echo -e "${YELLOW}💡 提示: 请先使用 'save' 命令保存SLAM地图${NC}"
            exit 1
        fi

        # 如果提供了参数，解析为绝对路径/相对路径/相对saved_maps或仅文件名
        if [ -n "${2-}" ]; then
            user_arg="${2}"
            # 1) 绝对或当前工作目录相对路径
            if [ -f "$user_arg" ]; then
                selected_pcd="$user_arg"
            else
                # 2) 去掉可能的 saved_maps/ 前缀后，按 saved_maps 下的相对子路径查找
                rel_arg="${user_arg#saved_maps/}"
                if [ -f "$MAPS_DIR/$rel_arg" ]; then
                    selected_pcd="$MAPS_DIR/$rel_arg"
                else
                    # 3) 使用索引快速查找（如仅给了 basename）
                    search_name="$(basename "$user_arg")"
                    INDEX_TOOL="$SCRIPT_DIR/pcd_index.py"
                    if [ -f "$INDEX_TOOL" ]; then
                        found_match=$(python3 "$INDEX_TOOL" --root "$MAPS_DIR" --index "$MAPS_DIR/.pcd_index.json" --ensure --find "$search_name" 2>/dev/null || true)
                    else
                        found_match=""
                    fi
                    if [ -n "$found_match" ]; then
                        selected_pcd="$found_match"
                    else
                        echo -e "${RED}❌ 指定的PCD文件不存在: $2${NC}"
                        echo -e "${YELLOW}ℹ 提示: 可传入以下任一形式${NC}"
                        echo "  • 绝对路径: /full/path/to/map.pcd"
                        echo "  • 相对当前目录: saved_maps/subdir/map.pcd"
                        echo "  • 相对 saved_maps 的子路径: subdir/map.pcd"
                        echo "  • 仅文件名(若唯一): map.pcd"
                        exit 1
                    fi
                fi
            fi
        else
            # 未提供参数时，再展示列表（避免每次都全量遍历）
            echo -e "${CYAN}📋 可用的PCD地图文件(示例):${NC}"
            INDEX_TOOL="$SCRIPT_DIR/pcd_index.py"
            if [ -f "$INDEX_TOOL" ]; then
                python3 "$INDEX_TOOL" --root "$MAPS_DIR" --ensure >/dev/null 2>&1 || true
                latest_pcd=$(python3 "$INDEX_TOOL" --root "$MAPS_DIR" --latest 2>/dev/null || true)
                if [ -n "$latest_pcd" ]; then
                    basename_file=$(basename "$latest_pcd")
                    size=$(ls -lh "$latest_pcd" | awk '{print $5}')
                    mtime=$(ls -l "$latest_pcd" | awk '{print $6, $7, $8}')
                    echo "  最新: $basename_file (大小: $size, 修改时间: $mtime)"
                fi
            else
                echo "  (索引工具缺失，跳过展示)"
            fi
            # 使用最新的PCD文件
            if [ -f "$INDEX_TOOL" ]; then
                selected_pcd=$(python3 "$INDEX_TOOL" --root "$MAPS_DIR" --latest 2>/dev/null || true)
            else
                selected_pcd=$(find "$MAPS_DIR" -name "*.pcd" -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
            fi
            if [ -z "$selected_pcd" ]; then
                echo -e "${RED}❌ 无法确定最新的PCD文件${NC}"
                exit 1
            fi
        fi

        echo -e "${YELLOW}🔄 正在转换PCD地图: $(basename "$selected_pcd")${NC}"

        # 设置输出文件名
        pcd_basename=$(basename "$selected_pcd" .pcd)
        output_file="$MAPS_DIR/${pcd_basename}_gridmap"

        # 透传额外参数（从第3个参数起），方便现场调参
        EXTRA_ARGS=("${@:3}")

        # 执行转换：默认更稳健的阈值与较小膨胀，避免室外高空与窄门被误阻塞
        # 可通过传入额外参数覆盖：如 --obstacle-thresh 10 --inflate-radius 0.10 --disable-panoramic
        if python3 "$SCRIPT_DIR/pcd_to_gridmap.py" "$selected_pcd" -o "$output_file" \
            --resolution 0.05 --robot-height 0.8 --inflate-radius 0.15 --obstacle-thresh 8 \
            --ground-dominates-ratio 2.0 "${EXTRA_ARGS[@]}"; then
            echo ""
            echo -e "${GREEN}✅ 转换完成!${NC}"
            echo -e "${CYAN}📁 输出文件:${NC}"
            ls -lh "$output_file".* 2>/dev/null | while read -r line; do
                echo "  $line"
            done

            echo ""
            echo -e "${YELLOW}💡 使用提示:${NC}"
            echo "  • PGM文件: 可在图像查看器中打开查看"
            echo "  • YAML文件: ROS导航栈使用的地图元数据"
            echo "  • INFO文件: 转换参数和统计信息"
            echo ""
            echo -e "${CYAN}🗺️  在ROS中加载栅格地图:${NC}"
            echo "  ros2 run map_server map_server --ros-args -p yaml_filename:=$output_file.yaml"
        else
            echo -e "${RED}❌ 转换失败${NC}"
            exit 1
        fi
        ;;

    "gridmap-gui")
        echo -e "${YELLOW}🖼️  启动栅格地图可视化转换器 (PyQt5)${NC}"
        if [ -f "$SCRIPT_DIR/gridmap_gui.py" ]; then
            python3 "$SCRIPT_DIR/gridmap_gui.py"
        else
            echo -e "${RED}❌ GUI脚本不存在: $SCRIPT_DIR/gridmap_gui.py${NC}"
            exit 1
        fi
        ;;

    "refine")
        echo -e "${YELLOW}🔧 地图精细化处理工具${NC}"
        echo -e "${CYAN}功能: 法向量ICP配准增强、平面结构保护、边缘特征强化、几何约束优化${NC}"
        echo ""

        # 检查精细化工具是否存在
        if [ ! -f "$SCRIPT_DIR/map_refinement.py" ]; then
            echo -e "${RED}❌ 精细化工具不存在: $SCRIPT_DIR/map_refinement.py${NC}"
            exit 1
        fi

        # 检查Python依赖
        if ! python3 -c "import open3d, numpy, sklearn, scipy" 2>/dev/null; then
            echo -e "${RED}❌ 缺少Python依赖包${NC}"
            echo -e "${YELLOW}请安装依赖: pip3 install open3d numpy scikit-learn scipy pyyaml${NC}"
            exit 1
        fi

        # 检查保存目录是否存在
        MAPS_DIR="$(dirname "$SCRIPT_DIR")/saved_maps"
        if [ ! -d "$MAPS_DIR" ]; then
            echo -e "${RED}❌ 地图保存目录不存在: $MAPS_DIR${NC}"
            echo -e "${YELLOW}💡 提示: 请先使用 'save' 命令保存SLAM地图${NC}"
            exit 1
        fi

        # 检查参数
        if [ -n "$2" ]; then
            # 处理指定文件
            INPUT_FILE="$2"
            if [ ! -f "$INPUT_FILE" ]; then
                echo -e "${RED}❌ 输入文件不存在: $INPUT_FILE${NC}"
                exit 1
            fi

            # 生成输出文件名
            INPUT_DIR=$(dirname "$INPUT_FILE")
            INPUT_BASE=$(basename "$INPUT_FILE" .pcd)
            OUTPUT_FILE="$INPUT_DIR/${INPUT_BASE}_refined.pcd"

            # 解析额外参数
            PIPELINE="full"
            SAVE_INTERMEDIATE=false
            QUALITY_REPORT=false
            DENOISING_MODE=""

            shift 2  # 跳过命令和文件名
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --pipeline)
                        PIPELINE="$2"
                        shift 2
                        ;;
                    --output)
                        OUTPUT_FILE="$2"
                        shift 2
                        ;;
                    --denoising-mode)
                        DENOISING_MODE="$2"
                        case $2 in
                            conservative|balanced|aggressive)
                                # 有效的去噪模式
                                ;;
                            *)
                                echo -e "${RED}❌ 无效的去噪模式: $2${NC}"
                                echo -e "${YELLOW}支持的模式: conservative, balanced, aggressive${NC}"
                                exit 1
                                ;;
                        esac
                        shift 2
                        ;;
                    --save-intermediate)
                        SAVE_INTERMEDIATE=true
                        shift
                        ;;
                    --quality-report)
                        QUALITY_REPORT=true
                        shift
                        ;;
                    *)
                        echo -e "${YELLOW}⚠️  未知参数: $1${NC}"
                        shift
                        ;;
                esac
            done

            echo -e "${CYAN}🔄 开始精细化处理...${NC}"
            echo "输入文件: $INPUT_FILE"
            echo "输出文件: $OUTPUT_FILE"
            echo "处理管道: $PIPELINE"
            if [ -n "$DENOISING_MODE" ]; then
                echo "去噪模式: $DENOISING_MODE"
            fi
            echo ""

            # 构建命令
            # 使用优化配置文件解决噪点问题
            OPTIMIZED_CONFIG="$SCRIPT_DIR/map_refinement/config/refinement_config_optimized.yaml"
            if [ -f "$OPTIMIZED_CONFIG" ]; then
                CMD="python3 $SCRIPT_DIR/map_refinement.py \"$INPUT_FILE\" --output \"$OUTPUT_FILE\" --pipeline $PIPELINE --config \"$OPTIMIZED_CONFIG\""
                echo -e "${CYAN}💡 使用优化配置（减少空旷区域噪点）${NC}"
            else
                CMD="python3 $SCRIPT_DIR/map_refinement.py \"$INPUT_FILE\" --output \"$OUTPUT_FILE\" --pipeline $PIPELINE"
                echo -e "${YELLOW}⚠️  使用默认配置${NC}"
            fi

            if [ -n "$DENOISING_MODE" ]; then
                CMD="$CMD --denoising-mode $DENOISING_MODE"
            fi

            if [ "$SAVE_INTERMEDIATE" = true ]; then
                CMD="$CMD --save-intermediate"
            fi

            if [ "$QUALITY_REPORT" = true ]; then
                CMD="$CMD --quality-report"
            fi

            # 执行精细化处理
            echo -e "${BLUE}执行命令: $CMD${NC}"
            if eval $CMD; then
                echo ""
                echo -e "${GREEN}✅ 精细化处理完成${NC}"

                # 显示结果文件信息
                if [ -f "$OUTPUT_FILE" ]; then
                    file_size=$(du -h "$OUTPUT_FILE" | cut -f1)
                    echo -e "${CYAN}📄 输出文件: $OUTPUT_FILE (大小: $file_size)${NC}"

                    # 检查质量报告
                    REPORT_FILE="${OUTPUT_FILE%.*}_quality_report.json"
                    if [ -f "$REPORT_FILE" ]; then
                        echo -e "${CYAN}📊 质量报告: $REPORT_FILE${NC}"
                    fi
                fi

                echo ""
                echo -e "${YELLOW}💡 使用提示:${NC}"
                echo "  • 查看结果: $0 view \"$OUTPUT_FILE\" --rviz"
                echo "  • 转换栅格地图: $0 gridmap (选择精细化后的地图)"
                echo "  • 批量处理: $0 refine batch saved_maps/*.pcd --output-dir refined/"
                echo "  • 保守去噪: $0 refine map.pcd --denoising-mode conservative"
                echo "  • 激进去噪: $0 refine map.pcd --denoising-mode aggressive"
            else
                echo ""
                echo -e "${RED}❌ 精细化处理失败${NC}"
                exit 1
            fi

        elif [ "$2" = "batch" ]; then
            # 批量处理模式
            shift 2  # 跳过 refine batch

            if [ $# -eq 0 ]; then
                echo -e "${RED}❌ 批量模式需要指定输入文件模式${NC}"
                echo -e "${YELLOW}示例: $0 refine batch saved_maps/*.pcd --output-dir refined/${NC}"
                exit 1
            fi

            # 解析参数
            OUTPUT_DIR="refined_maps"
            PIPELINE="full"
            DENOISING_MODE=""

            # 提取文件模式
            FILE_PATTERN=""
            while [[ $# -gt 0 ]] && [[ "$1" != --* ]]; do
                if [ -z "$FILE_PATTERN" ]; then
                    FILE_PATTERN="$1"
                else
                    FILE_PATTERN="$FILE_PATTERN $1"
                fi
                shift
            done

            # 解析选项
            while [[ $# -gt 0 ]]; do
                case $1 in
                    --output-dir)
                        OUTPUT_DIR="$2"
                        shift 2
                        ;;
                    --pipeline)
                        PIPELINE="$2"
                        shift 2
                        ;;
                    --denoising-mode)
                        DENOISING_MODE="$2"
                        case $2 in
                            conservative|balanced|aggressive)
                                # 有效的去噪模式
                                ;;
                            *)
                                echo -e "${RED}❌ 无效的去噪模式: $2${NC}"
                                echo -e "${YELLOW}支持的模式: conservative, balanced, aggressive${NC}"
                                exit 1
                                ;;
                        esac
                        shift 2
                        ;;
                    *)
                        echo -e "${YELLOW}⚠️  未知参数: $1${NC}"
                        shift
                        ;;
                esac
            done

            echo -e "${CYAN}🔄 开始批量精细化处理...${NC}"
            echo "文件模式: $FILE_PATTERN"
            echo "输出目录: $OUTPUT_DIR"
            echo "处理管道: $PIPELINE"
            if [ -n "$DENOISING_MODE" ]; then
                echo "去噪模式: $DENOISING_MODE"
            fi
            echo ""

            # 执行批量处理
            # 使用优化配置文件解决噪点问题
            OPTIMIZED_CONFIG="$SCRIPT_DIR/map_refinement/config/refinement_config_optimized.yaml"
            if [ -f "$OPTIMIZED_CONFIG" ]; then
                CMD="python3 $SCRIPT_DIR/map_refinement.py batch_refine $FILE_PATTERN --output-dir \"$OUTPUT_DIR\" --pipeline $PIPELINE --config \"$OPTIMIZED_CONFIG\""
                echo -e "${CYAN}💡 使用优化配置（减少空旷区域噪点）${NC}"
            else
                CMD="python3 $SCRIPT_DIR/map_refinement.py batch_refine $FILE_PATTERN --output-dir \"$OUTPUT_DIR\" --pipeline $PIPELINE"
                echo -e "${YELLOW}⚠️  使用默认配置${NC}"
            fi

            if [ -n "$DENOISING_MODE" ]; then
                CMD="$CMD --denoising-mode $DENOISING_MODE"
            fi

            echo -e "${BLUE}执行命令: $CMD${NC}"
            if eval $CMD; then
                echo ""
                echo -e "${GREEN}✅ 批量精细化处理完成${NC}"
                echo -e "${CYAN}📁 结果目录: $OUTPUT_DIR${NC}"
            else
                echo ""
                echo -e "${RED}❌ 批量精细化处理失败${NC}"
                exit 1
            fi

        else
            # 显示可用的PCD文件并提供交互选择
            echo -e "${CYAN}📋 可用的PCD地图文件:${NC}"
            pcd_files=()
            i=1
            while IFS= read -r -d '' file; do
                echo "  $i) $(basename "$file")"
                pcd_files+=("$file")
                ((i++))
            done < <(find "$MAPS_DIR" -name "*.pcd" -print0 | sort -z)

            if [ ${#pcd_files[@]} -eq 0 ]; then
                echo -e "${RED}❌ 未找到PCD文件${NC}"
                echo -e "${YELLOW}💡 提示: 请先保存SLAM地图${NC}"
                exit 1
            fi

            echo ""
            echo -e "${YELLOW}请选择要精细化的PCD文件 (输入数字):${NC} "
            read -r choice

            if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le ${#pcd_files[@]} ]; then
                selected_file="${pcd_files[$((choice-1))]}"

                echo ""
                echo -e "${CYAN}🔧 精细化管道选项:${NC}"
                echo "  1) full     - 完整处理 (去噪+结构保护+边缘增强+几何优化+表面重建+细节保护+密度均匀化)"
                echo "  2) basic    - 基础处理 (仅去噪+结构保护)"
                echo "  3) denoise  - 仅去噪处理 (推荐先选择去噪模式)"
                echo "  4) structure- 结构保护处理"
                echo "  5) edges    - 边缘特征增强"
                echo ""
                echo -e "${YELLOW}请选择处理管道 (直接回车默认使用 full):${NC} "
                read -r pipeline_choice

                case "$pipeline_choice" in
                    "1"|"") PIPELINE="full" ;;
                    "2") PIPELINE="basic" ;;
                    "3") PIPELINE="denoise" ;;
                    "4") PIPELINE="structure" ;;
                    "5") PIPELINE="edges" ;;
                    *)
                        echo -e "${YELLOW}使用默认管道: full${NC}"
                        PIPELINE="full"
                        ;;
                esac

                # 选择去噪模式（如果管道包含去噪）
                DENOISING_MODE=""
                if [[ "$PIPELINE" == "full" || "$PIPELINE" == "basic" || "$PIPELINE" == "denoise" ]]; then
                    echo ""
                    echo -e "${CYAN}🔧 去噪模式选项:${NC}"
                    echo "  1) conservative - 保守模式 (保留更多细节，推荐)"
                    echo "  2) balanced     - 平衡模式 (默认)"
                    echo "  3) aggressive   - 激进模式 (严格去噪，可能丢失细节)"
                    echo ""
                    echo -e "${YELLOW}请选择去噪模式 (直接回车默认使用 conservative):${NC} "
                    read -r denoising_choice

                    case "$denoising_choice" in
                        "1"|"") DENOISING_MODE="conservative" ;;
                        "2") DENOISING_MODE="balanced" ;;
                        "3") DENOISING_MODE="aggressive" ;;
                        *)
                            echo -e "${YELLOW}使用默认去噪模式: conservative${NC}"
                            DENOISING_MODE="conservative"
                            ;;
                    esac
                fi

                # 生成输出文件名
                INPUT_DIR=$(dirname "$selected_file")
                INPUT_BASE=$(basename "$selected_file" .pcd)
                OUTPUT_FILE="$INPUT_DIR/${INPUT_BASE}_refined.pcd"

                echo ""
                echo -e "${CYAN}🔄 开始精细化处理...${NC}"
                echo "输入文件: $(basename "$selected_file")"
                echo "输出文件: $(basename "$OUTPUT_FILE")"
                echo "处理管道: $PIPELINE"
                if [ -n "$DENOISING_MODE" ]; then
                    echo "去噪模式: $DENOISING_MODE"
                fi
                echo ""

                # 构建命令
                # 使用优化配置文件解决噪点问题
                OPTIMIZED_CONFIG="$SCRIPT_DIR/map_refinement/config/refinement_config_optimized.yaml"
                if [ -f "$OPTIMIZED_CONFIG" ]; then
                    CMD="python3 \"$SCRIPT_DIR/map_refinement.py\" \"$selected_file\" --output \"$OUTPUT_FILE\" --pipeline \"$PIPELINE\" --quality-report --config \"$OPTIMIZED_CONFIG\""
                    echo -e "${CYAN}💡 使用优化配置（减少空旷区域噪点）${NC}"
                else
                    CMD="python3 \"$SCRIPT_DIR/map_refinement.py\" \"$selected_file\" --output \"$OUTPUT_FILE\" --pipeline \"$PIPELINE\" --quality-report"
                    echo -e "${YELLOW}⚠️  使用默认配置${NC}"
                fi
                if [ -n "$DENOISING_MODE" ]; then
                    CMD="$CMD --denoising-mode $DENOISING_MODE"
                fi

                # 执行精细化处理
                if eval $CMD; then
                    echo ""
                    echo -e "${GREEN}✅ 精细化处理完成${NC}"

                    # 显示结果
                    if [ -f "$OUTPUT_FILE" ]; then
                        file_size=$(du -h "$OUTPUT_FILE" | cut -f1)
                        echo -e "${CYAN}📄 精细化地图: $(basename "$OUTPUT_FILE") (大小: $file_size)${NC}"

                        # 检查质量报告
                        REPORT_FILE="${OUTPUT_FILE%.*}_quality_report.json"
                        if [ -f "$REPORT_FILE" ]; then
                            echo -e "${CYAN}📊 质量报告: $(basename "$REPORT_FILE")${NC}"
                        fi
                    fi

                    echo ""
                    echo -e "${YELLOW}💡 后续操作:${NC}"
                    echo "  • 查看结果: $0 view \"$OUTPUT_FILE\" --rviz"
                    echo "  • 转换栅格地图: $0 gridmap"
                else
                    echo ""
                    echo -e "${RED}❌ 精细化处理失败${NC}"
                    exit 1
                fi
            else
                echo -e "${RED}❌ 无效的选择${NC}"
                exit 1
            fi
        fi
        ;;

    "benchmark")
        echo -e "${CYAN}🔬 地图精细化性能基准测试${NC}"
        echo ""

        # 检查Python依赖
        if ! python3 -c "import open3d, numpy, sklearn" 2>/dev/null; then
            echo -e "${RED}❌ 缺少Python依赖包${NC}"
            echo -e "${YELLOW}请安装依赖: pip3 install open3d numpy scikit-learn${NC}"
            exit 1
        fi

        # 检查基准测试脚本
        if [ ! -f "$SCRIPT_DIR/benchmark_refinement.py" ]; then
            echo -e "${RED}❌ 基准测试脚本不存在: $SCRIPT_DIR/benchmark_refinement.py${NC}"
            exit 1
        fi

        # 解析参数
        TEST_TYPE=""
        CONFIG_FILE=""

        shift  # 跳过benchmark命令
        while [[ $# -gt 0 ]]; do
            case $1 in
                --quick)
                    TEST_TYPE="--quick"
                    shift
                    ;;
                --full)
                    TEST_TYPE="--full"
                    shift
                    ;;
                --config)
                    CONFIG_FILE="$2"
                    shift 2
                    ;;
                *)
                    echo -e "${YELLOW}⚠️  未知参数: $1${NC}"
                    shift
                    ;;
            esac
        done

        echo -e "${CYAN}📋 性能测试选项:${NC}"
        echo "  • 快速测试: 较小数据集，快速了解性能"
        echo "  • 标准测试: 中等数据集，综合评估 (默认)"
        echo "  • 完整测试: 大数据集，详细性能分析"
        echo ""

        if [ -z "$TEST_TYPE" ]; then
            echo -e "${YELLOW}请选择测试类型:${NC}"
            echo "  1) 快速测试 (推荐)"
            echo "  2) 标准测试"
            echo "  3) 完整测试"
            echo ""
            echo -e "${YELLOW}请选择 (直接回车默认标准测试):${NC} "
            read -r choice

            case "$choice" in
                "1") TEST_TYPE="--quick" ;;
                "2"|"") TEST_TYPE="" ;;
                "3") TEST_TYPE="--full" ;;
                *)
                    echo -e "${YELLOW}使用默认选项: 标准测试${NC}"
                    TEST_TYPE=""
                    ;;
            esac
        fi

        # 构建命令
        CMD="python3 $SCRIPT_DIR/benchmark_refinement.py"

        if [ -n "$TEST_TYPE" ]; then
            CMD="$CMD $TEST_TYPE"
        fi

        if [ -n "$CONFIG_FILE" ]; then
            if [ -f "$CONFIG_FILE" ]; then
                CMD="$CMD --config $CONFIG_FILE"
            else
                echo -e "${YELLOW}⚠️  配置文件不存在，使用默认配置: $CONFIG_FILE${NC}"
            fi
        fi

        echo -e "${BLUE}执行命令: $CMD${NC}"
        echo ""

        # 执行性能测试
        if eval $CMD; then
            echo ""
            echo -e "${GREEN}✅ 性能基准测试完成${NC}"

            # 查找结果目录
            RESULTS_DIR="$(dirname "$SCRIPT_DIR")/benchmark_results"
            if [ -d "$RESULTS_DIR" ]; then
                echo -e "${CYAN}📊 测试结果位置: $RESULTS_DIR${NC}"
                echo ""
                echo -e "${YELLOW}💡 查看结果:${NC}"
                echo "  • 性能报告: cat $RESULTS_DIR/performance_report.md"
                echo "  • 详细数据: ls $RESULTS_DIR/*.json"
            fi
        else
            echo ""
            echo -e "${RED}❌ 性能基准测试失败${NC}"
            exit 1
        fi
        ;;

    "filter")
        case "$2" in
            "test")
                echo -e "${YELLOW}🧪 运行动态对象过滤器测试...${NC}"
                if [ -f "$SCRIPT_DIR/test_dynamic_filter.sh" ]; then
                    shift 2  # 移除 filter test
                    bash "$SCRIPT_DIR/test_dynamic_filter.sh" "$@"
                else
                    echo -e "${RED}❌ 测试脚本不存在: $SCRIPT_DIR/test_dynamic_filter.sh${NC}"
                    exit 1
                fi
                ;;

            "viz"|"visualize")
                echo -e "${CYAN}🎯 启动动态过滤器可视化工具...${NC}"
                if [ -f "$SCRIPT_DIR/visualize_dynamic_filter.py" ]; then
                    load_ros_env
                    echo -e "${BLUE}启动Python可视化工具...${NC}"
                    python3 "$SCRIPT_DIR/visualize_dynamic_filter.py" &
                    VIZ_PID=$!

                    echo -e "${BLUE}启动RViz可视化...${NC}"
                    sleep 2
                    RVIZ_CONFIG="ws_livox/src/localizer/rviz/localizer.rviz"
                    if [ -f "$RVIZ_CONFIG" ]; then
                        rviz2 -d "$RVIZ_CONFIG" &
                        RVIZ_PID=$!
                    else
                        rviz2 &
                        RVIZ_PID=$!
                    fi

                    echo -e "${GREEN}✅ 可视化工具已启动${NC}"
                    echo -e "${YELLOW}💡 Python界面：参数调优和统计监控${NC}"
                    echo -e "${YELLOW}💡 RViz界面：3D点云可视化${NC}"
                    echo -e "${YELLOW}⚠️  按Ctrl+C停止所有可视化工具${NC}"

                    trap "echo ''; echo '🛑 停止可视化工具...'; kill $VIZ_PID 2>/dev/null; kill $RVIZ_PID 2>/dev/null; exit 0" SIGINT
                    wait
                else
                    echo -e "${RED}❌ 可视化工具不存在: $SCRIPT_DIR/visualize_dynamic_filter.py${NC}"
                    exit 1
                fi
                ;;

            "tune")
                echo -e "${CYAN}🔧 动态过滤器参数调优...${NC}"
                if [ -f "$SCRIPT_DIR/visualize_dynamic_filter.py" ]; then
                    load_ros_env
                    echo -e "${BLUE}启动参数调优界面...${NC}"
                    python3 "$SCRIPT_DIR/visualize_dynamic_filter.py" --tune-mode
                else
                    echo -e "${RED}❌ 调优工具不存在: $SCRIPT_DIR/visualize_dynamic_filter.py${NC}"
                    exit 1
                fi
                ;;

            "status")
                echo -e "${CYAN}📊 动态对象过滤器状态检查${NC}"
                echo "====================================="

                load_ros_env

                # 检查localizer节点
                if ros2 node list 2>/dev/null | grep -q "localizer_node"; then
                    echo -e "  Localizer节点: ${GREEN}✅ 运行中${NC}"

                    # 检查过滤器话题
                    filter_topics=(
                        "/localizer/filtered_cloud:过滤后点云"
                        "/localizer/filter_stats:过滤统计"
                        "/localizer/dynamic_cloud:动态点云"
                    )

                    echo ""
                    echo -e "${YELLOW}过滤器话题状态:${NC}"
                    for topic_info in "${filter_topics[@]}"; do
                        topic="${topic_info%%:*}"
                        description="${topic_info##*:}"

                        if timeout 2s ros2 topic hz "$topic" --once 2>/dev/null | grep -q "Hz"; then
                            echo -e "  $description: ${GREEN}✅ 活跃${NC}"
                        else
                            echo -e "  $description: ${YELLOW}⚠️  无数据${NC}"
                        fi
                    done

                    # 检查配置状态
                    echo ""
                    echo -e "${YELLOW}配置文件状态:${NC}"
                    CONFIG_FILE="ws_livox/src/localizer/config/localizer.yaml"
                    if [ -f "$CONFIG_FILE" ]; then
                        if grep -q "dynamic_filter:" "$CONFIG_FILE"; then
                            if grep -A1 "dynamic_filter:" "$CONFIG_FILE" | grep -q "enable: true"; then
                                echo -e "  动态过滤器: ${GREEN}✅ 已启用${NC}"
                            else
                                echo -e "  动态过滤器: ${YELLOW}⏸  已配置但未启用${NC}"
                            fi
                        else
                            echo -e "  动态过滤器: ${RED}❌ 未配置${NC}"
                        fi
                    else
                        echo -e "  配置文件: ${RED}❌ 不存在${NC}"
                    fi

                else
                    echo -e "  Localizer节点: ${RED}❌ 未运行${NC}"
                    echo -e "  提示: 运行 '$0 start' 启动SLAM系统"
                fi

                # 检查源代码状态
                echo ""
                echo -e "${YELLOW}源代码状态:${NC}"
                if [ -f "ws_livox/src/localizer/src/localizers/dynamic_object_filter.cpp" ]; then
                    echo -e "  过滤器实现: ${GREEN}✅ 已集成${NC}"
                else
                    echo -e "  过滤器实现: ${RED}❌ 未找到${NC}"
                fi

                if [ -f "ws_livox/src/localizer/src/localizers/dynamic_object_filter.h" ]; then
                    echo -e "  过滤器头文件: ${GREEN}✅ 存在${NC}"
                else
                    echo -e "  过滤器头文件: ${RED}❌ 未找到${NC}"
                fi
                ;;

            "enable")
                echo -e "${GREEN}🔓 启用动态对象过滤器...${NC}"
                CONFIG_FILE="ws_livox/src/localizer/config/localizer.yaml"

                if [ -f "$CONFIG_FILE" ]; then
                    if grep -q "dynamic_filter:" "$CONFIG_FILE"; then
                        # 更新enable状态
                        sed -i '/dynamic_filter:/,/^[[:space:]]*[^[:space:]]/ s/enable: false/enable: true/' "$CONFIG_FILE"
                        sed -i '/dynamic_filter:/,/^[[:space:]]*[^[:space:]]/ s/enable:false/enable: true/' "$CONFIG_FILE"
                        echo -e "${GREEN}✅ 动态过滤器已启用${NC}"
                        echo -e "${YELLOW}💡 重启localizer以应用更改: $0 restart${NC}"
                    else
                        echo -e "${RED}❌ 配置文件中未找到动态过滤器配置${NC}"
                        exit 1
                    fi
                else
                    echo -e "${RED}❌ 配置文件不存在: $CONFIG_FILE${NC}"
                    exit 1
                fi
                ;;

            "disable")
                echo -e "${YELLOW}🔒 禁用动态对象过滤器...${NC}"
                CONFIG_FILE="ws_livox/src/localizer/config/localizer.yaml"

                if [ -f "$CONFIG_FILE" ]; then
                    if grep -q "dynamic_filter:" "$CONFIG_FILE"; then
                        # 更新enable状态
                        sed -i '/dynamic_filter:/,/^[[:space:]]*[^[:space:]]/ s/enable: true/enable: false/' "$CONFIG_FILE"
                        echo -e "${YELLOW}⏸  动态过滤器已禁用${NC}"
                        echo -e "${YELLOW}💡 重启localizer以应用更改: $0 restart${NC}"
                    else
                        echo -e "${RED}❌ 配置文件中未找到动态过滤器配置${NC}"
                        exit 1
                    fi
                else
                    echo -e "${RED}❌ 配置文件不存在: $CONFIG_FILE${NC}"
                    exit 1
                fi
                ;;

            "help"|"")
                echo -e "${CYAN}动态对象过滤器功能帮助:${NC}"
                echo "=================================="
                echo "用法: $0 filter [子命令]"
                echo ""
                echo "可用子命令:"
                echo -e "  ${YELLOW}test${NC}      - 运行过滤器集成测试"
                echo -e "  ${YELLOW}viz${NC}       - 启动可视化工具（Python + RViz）"
                echo -e "  ${YELLOW}tune${NC}      - 启动参数调优界面"
                echo -e "  ${YELLOW}status${NC}    - 检查过滤器状态"
                echo -e "  ${YELLOW}enable${NC}    - 启用动态过滤"
                echo -e "  ${YELLOW}disable${NC}   - 禁用动态过滤"
                echo -e "  ${YELLOW}help${NC}      - 显示此帮助信息"
                echo ""
                echo "示例:"
                echo "  $0 filter test     # 运行测试"
                echo "  $0 filter viz      # 启动可视化"
                echo "  $0 filter status   # 检查状态"
                ;;

            *)
                echo -e "${RED}未知的filter子命令: $2${NC}"
                echo "使用 '$0 filter help' 查看可用命令"
                exit 1
                ;;
        esac
        ;;

    "test")
        case "$2" in
            "compile")
                echo -e "${YELLOW}🔨 运行编译测试...${NC}"
                cd ws_livox
                colcon build --packages-select localizer
                ;;

            "unit")
                echo -e "${YELLOW}🧪 运行单元测试...${NC}"
                cd ws_livox
                colcon test --packages-select localizer
                if [ $? -eq 0 ]; then
                    echo -e "${GREEN}✅ 单元测试通过${NC}"
                else
                    echo -e "${RED}❌ 单元测试失败${NC}"
                    colcon test-result --verbose
                fi
                ;;

            "integration")
                echo -e "${YELLOW}🔗 运行集成测试...${NC}"
                if [ -f "$SCRIPT_DIR/test_dynamic_filter.sh" ]; then
                    bash "$SCRIPT_DIR/test_dynamic_filter.sh" full
                else
                    echo -e "${RED}❌ 集成测试脚本不存在${NC}"
                    exit 1
                fi
                ;;

            "filter")
                echo -e "${YELLOW}🎯 运行动态过滤器专项测试...${NC}"
                if [ -f "$SCRIPT_DIR/test_dynamic_filter.sh" ]; then
                    bash "$SCRIPT_DIR/test_dynamic_filter.sh" quick true
                else
                    echo -e "${RED}❌ 过滤器测试脚本不存在${NC}"
                    exit 1
                fi
                ;;

            "all")
                echo -e "${CYAN}🎯 运行全套测试...${NC}"
                echo ""

                echo -e "${BLUE}1/4 编译测试${NC}"
                cd ws_livox && colcon build --packages-select localizer

                echo ""
                echo -e "${BLUE}2/4 单元测试${NC}"
                colcon test --packages-select localizer

                echo ""
                echo -e "${BLUE}3/4 动态过滤器测试${NC}"
                cd ..
                if [ -f "$SCRIPT_DIR/test_dynamic_filter.sh" ]; then
                    bash "$SCRIPT_DIR/test_dynamic_filter.sh" quick
                fi

                echo ""
                echo -e "${BLUE}4/4 集成测试${NC}"
                if [ -f "$SCRIPT_DIR/test_dynamic_filter.sh" ]; then
                    bash "$SCRIPT_DIR/test_dynamic_filter.sh" full
                fi

                echo ""
                echo -e "${GREEN}✅ 全套测试完成${NC}"
                ;;

            "help"|"")
                echo -e "${CYAN}测试功能帮助:${NC}"
                echo "================"
                echo "用法: $0 test [测试类型]"
                echo ""
                echo "可用测试类型:"
                echo -e "  ${YELLOW}compile${NC}     - 编译测试"
                echo -e "  ${YELLOW}unit${NC}        - 单元测试"
                echo -e "  ${YELLOW}integration${NC} - 集成测试"
                echo -e "  ${YELLOW}filter${NC}      - 动态过滤器专项测试"
                echo -e "  ${YELLOW}all${NC}         - 运行全套测试"
                echo ""
                echo "示例:"
                echo "  $0 test compile    # 仅编译测试"
                echo "  $0 test filter     # 仅过滤器测试"
                echo "  $0 test all        # 全套测试"
                ;;

            *)
                echo -e "${RED}未知的测试类型: $2${NC}"
                echo "使用 '$0 test help' 查看可用测试"
                exit 1
                ;;
        esac
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
