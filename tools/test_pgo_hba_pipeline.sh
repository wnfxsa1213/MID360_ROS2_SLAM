#!/usr/bin/env bash

# 这个SB脚本用来整体验证 PGO → HBA → FAST-LIO 的协同链路。
# 用法：./tools/test_pgo_hba_pipeline.sh [rosbag目录] [launch额外参数...]
# 若不传路径则默认 data/rosbags/slam_data_20251016_112809。

set -euo pipefail

trap 'cleanup' EXIT INT TERM

cleanup() {
    if [[ -n "${LAUNCH_PID:-}" ]] && ps -p "${LAUNCH_PID}" >/dev/null 2>&1; then
        echo "[test] 停止cooperative_slam_system.launch(py) (pid=${LAUNCH_PID})"
        kill "${LAUNCH_PID}" >/dev/null 2>&1 || true
        wait "${LAUNCH_PID}" >/dev/null 2>&1 || true
    fi
}

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ws_livox"
DEFAULT_BAG_DIR="${ROOT_DIR}/data/rosbags/slam_data_20251016_112809"
LOG_DIR="${ROOT_DIR}/log/integration"
mkdir -p "${LOG_DIR}"

BAG_DIR="${1:-${DEFAULT_BAG_DIR}}"
shift || true

if [[ ! -f "${BAG_DIR}/metadata.yaml" ]]; then
    echo "[test][ERROR] 找不到rosbag metadata: ${BAG_DIR}/metadata.yaml" >&2
    exit 1
fi

if [[ ! -d "${WS_DIR}/install" ]]; then
    echo "[test][ERROR] 先跑一遍 colcon build，install目录都没生成你想咋玩？" >&2
    exit 1
fi

# 上下文准备
export COLCON_TRACE="${COLCON_TRACE:-}"
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-}"
export COLCON_PREFIX_PATH="${COLCON_PREFIX_PATH:-}"
export COLCON_PYTHON_EXECUTABLE="${COLCON_PYTHON_EXECUTABLE:-}"
set +u
source "${WS_DIR}/install/setup.bash"
set -u

echo "[test] 启动 cooperative_slam_system.launch.py"
ros2 launch fastlio2 cooperative_slam_system.launch.py \
    enable_hba_incremental:=true \
    "${@}" \
    >"${LOG_DIR}/cooperative_launch.log" 2>&1 &
LAUNCH_PID=$!
sleep 3

# 等待关键节点起来
echo "[test] 等待 /coordinator/metrics 话题..."
for i in {1..20}; do
    if ros2 topic info /coordinator/metrics >/dev/null 2>&1; then
        break
    fi
    if [[ ${i} -eq 20 ]]; then
        echo "[test][ERROR] /coordinator/metrics 话题迟迟没冒头，估计launch跪了" >&2
        exit 1
    fi
    sleep 1
done

echo "[test] 开始回放 rosbag: ${BAG_DIR}"
ros2 bag play "${BAG_DIR}" --clock -r 1.0 \
    >"${LOG_DIR}/rosbag_play.log" 2>&1 || {
    echo "[test][ERROR] rosbag play 异常退出" >&2
    exit 1
}

echo "[test] 抓取协同指标一次..."
if ! timeout 30 ros2 topic echo /coordinator/metrics --once \
        >"${LOG_DIR}/coordinator_metrics_sample.log" 2>&1; then
    echo "[test][ERROR] 30秒内没有等到 /coordinator/metrics，链路可能失效" >&2
    exit 1
fi

echo "[test] 抓取 HBA 输出点云一次..."
if ! timeout 30 ros2 topic echo /hba/map_points --once \
        >"${LOG_DIR}/hba_map_sample.log" 2>&1; then
    echo "[test][ERROR] 30秒内没有等到 /hba/map_points，增量HBA可能没跑起来" >&2
    exit 1
fi

echo "[test] 流水线验证完成 ✅ 日志在 ${LOG_DIR}"
