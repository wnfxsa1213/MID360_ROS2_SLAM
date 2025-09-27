#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
根据场景一键应用配置预设，并调用配置生成器：
- realtime 实机扫描：
  * pgo.cloud_topic = /slam/body_cloud
  * localizer.dynamic_filter.enable = false

- replay 回放复现：
  * pgo.cloud_topic = /localizer/filtered_cloud
  * localizer.dynamic_filter.enable = true
  * 放宽PGO回环参数（可选）：loop_search_radius、loop_time_tresh、loop_score_tresh、min_loop_detect_duration

用法：
  python3 tools/apply_preset.py --mode realtime
  python3 tools/apply_preset.py --mode replay
"""

import argparse
import sys
from pathlib import Path
import yaml
import subprocess


def load_yaml(p: Path):
    with p.open('r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def save_yaml(p: Path, data):
    with p.open('w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def apply_realtime(mc: dict) -> list:
    changes = []
    # 确保层级存在
    mc.setdefault('pgo', {})
    mc.setdefault('localizer', {})
    mc.setdefault('fastlio2', {})
    mc.setdefault('topics', {})
    mc['localizer'].setdefault('dynamic_filter', {})

    # 设置参数
    if mc['pgo'].get('cloud_topic') != '/slam/body_cloud':
        mc['pgo']['cloud_topic'] = '/slam/body_cloud'
        changes.append("pgo.cloud_topic=/slam/body_cloud")

    # 关闭 Localizer 动态过滤
    if mc['localizer']['dynamic_filter'].get('enable') is not False:
        mc['localizer']['dynamic_filter']['enable'] = False
        changes.append("localizer.dynamic_filter.enable=false")

    # FastLIO2 在实机通过过滤桥吃已过滤的话题
    # 统一顶层topics（供生成器使用）
    if mc['topics'].get('lidar') != '/livox/lidar_filtered':
        mc['topics']['lidar'] = '/livox/lidar_filtered'
        changes.append("topics.lidar=/livox/lidar_filtered")
    if mc['topics'].get('imu') != '/livox/imu':
        mc['topics']['imu'] = '/livox/imu'
        changes.append("topics.imu=/livox/imu")
    # 冗余写入 fastlio2.*（供其他工具参考）
    mc['fastlio2']['lidar_topic'] = mc['topics']['lidar']
    mc['fastlio2']['imu_topic'] = mc['topics']['imu']

    return changes


def apply_replay(mc: dict) -> list:
    changes = []
    mc.setdefault('pgo', {})
    mc.setdefault('localizer', {})
    mc.setdefault('fastlio2', {})
    mc.setdefault('topics', {})
    mc['localizer'].setdefault('dynamic_filter', {})

    # 让PGO吃过滤后的点云
    if mc['pgo'].get('cloud_topic') != '/localizer/filtered_cloud':
        mc['pgo']['cloud_topic'] = '/localizer/filtered_cloud'
        changes.append("pgo.cloud_topic=/localizer/filtered_cloud")

    # 开启 Localizer 动态过滤
    if mc['localizer']['dynamic_filter'].get('enable') is not True:
        mc['localizer']['dynamic_filter']['enable'] = True
        changes.append("localizer.dynamic_filter.enable=true")

    # 回放模式禁用过滤桥，FastLIO2 直接吃原始 /livox/lidar
    # 统一顶层topics（供生成器使用）
    if mc['topics'].get('lidar') != '/livox/lidar':
        mc['topics']['lidar'] = '/livox/lidar'
        changes.append("topics.lidar=/livox/lidar")
    if mc['topics'].get('imu') != '/livox/imu':
        mc['topics']['imu'] = '/livox/imu'
        changes.append("topics.imu=/livox/imu")
    # 冗余写入 fastlio2.*（供其他工具参考）
    mc['fastlio2']['lidar_topic'] = mc['topics']['lidar']
    mc['fastlio2']['imu_topic'] = mc['topics']['imu']

    # 放宽回放场景的PGO参数（可选，更易触发回环）
    # 仅在用户未主动设置为更极端值时才覆盖
    def set_if_missing_or_less(key_path, value, cmp='set'):
        cur = mc['pgo'].get(key_path)
        if cur is None:
            mc['pgo'][key_path] = value
            changes.append(f"pgo.{key_path}={value}")
        else:
            if cmp == 'gt' and not (cur >= value):
                mc['pgo'][key_path] = value
                changes.append(f"pgo.{key_path}={value}")
            elif cmp == 'lt' and not (cur <= value):
                mc['pgo'][key_path] = value
                changes.append(f"pgo.{key_path}={value}")

    # 半径更大（更容易命中）
    set_if_missing_or_less('loop_search_radius', 5.0, cmp='gt')
    # 时间阈值更短（更快尝试闭环）
    set_if_missing_or_less('loop_time_tresh', 10.0, cmp='lt')
    # 分数阈值放宽（更易通过）
    set_if_missing_or_less('loop_score_tresh', 0.30, cmp='gt')
    # 检测间隔缩短
    set_if_missing_or_less('min_loop_detect_duration', 2.0, cmp='lt')

    return changes


def main():
    parser = argparse.ArgumentParser(description='应用SLAM配置预设并生成配置')
    parser.add_argument('--mode', required=True, choices=['realtime', 'replay'], help='选择预设')
    parser.add_argument('--master-config', default='config/master_config.yaml', help='主配置文件路径')
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    mc_path = (repo_root / args.master_config).resolve()
    if not mc_path.exists():
        print(f"❌ 主配置文件不存在: {mc_path}")
        return 1

    mc = load_yaml(mc_path)
    if args.mode == 'realtime':
        changes = apply_realtime(mc)
    else:
        changes = apply_replay(mc)

    if not changes:
        print("ℹ️ 配置与该预设已一致，无需修改。")
    else:
        save_yaml(mc_path, mc)
        print("✅ 已应用预设:")
        for c in changes:
            print(f"  - {c}")

    # 生成所有配置
    gen = subprocess.run(['python3', str(repo_root / 'tools/config_generator.py')], cwd=str(repo_root))
    return gen.returncode


if __name__ == '__main__':
    sys.exit(main())
