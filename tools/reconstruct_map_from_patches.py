#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
离线重建地图：使用 patches + poses 重建整图

常见用法
- 使用 PGO 导出的目录（含 patches/ 与 poses.txt）直接重建：
    python3 tools/reconstruct_map_from_patches.py \
        --pgo-dir saved_maps/run1_pgo \
        --output saved_maps/run1_pgo_reconstructed.pcd

- 用 HBA 导出的位姿替换 PGO 的 poses：
    python3 tools/reconstruct_map_from_patches.py \
        --pgo-dir saved_maps/run1_pgo \
        --poses saved_maps/run1_hba_poses.txt \
        --output saved_maps/run1_pgo_hba_map.pcd

说明
- 支持两类位姿文件格式：
  1) PGO poses.txt:  每行 "patch_name.pcd tx ty tz qw qx qy qz"
  2) 简单位姿列表: 每行 "tx ty tz qw qx qy qz"，与 patches 按序对齐
- patches 目录中的文件名若为数字序号（例如 0.pcd, 1.pcd ...），将按数字排序对齐
"""

import argparse
import os
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import open3d as o3d


def parse_poses_file(poses_path: Path) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], List[Tuple[np.ndarray, np.ndarray]]]:
    """解析位姿文件。

    返回:
      - name_to_pose: 如果文件包含 patch 文件名，则返回 name -> (t, q) 的映射；否则为空字典
      - list_poses:   若为纯位姿列表（无文件名），返回 [(t, q), ...]；否则为空列表
    """
    name_to_pose: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    list_poses: List[Tuple[np.ndarray, np.ndarray]] = []
    with poses_path.open('r') as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            # PGO 格式：<name> tx ty tz qw qx qy qz
            if len(parts) >= 8 and parts[0].lower().endswith('.pcd'):
                name = parts[0]
                try:
                    tx, ty, tz, qw, qx, qy, qz = map(float, parts[1:8])
                except ValueError:
                    continue
                name_to_pose[name] = (np.array([tx, ty, tz], dtype=np.float64),
                                      np.array([qw, qx, qy, qz], dtype=np.float64))
            else:
                # 兜底：尝试读取最后 7 个为位姿
                if len(parts) >= 7:
                    try:
                        vals = list(map(float, parts[-7:]))
                        tx, ty, tz, qw, qx, qy, qz = vals
                        list_poses.append((np.array([tx, ty, tz], dtype=np.float64),
                                           np.array([qw, qx, qy, qz], dtype=np.float64)))
                    except ValueError:
                        continue
    return name_to_pose, list_poses


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """将四元数 [qw, qx, qy, qz] 转换为 3x3 旋转矩阵。"""
    qw, qx, qy, qz = q
    # 归一化
    norm = np.linalg.norm(q)
    if norm == 0:
        return np.eye(3)
    qw, qx, qy, qz = q / norm
    # 旋转矩阵
    R = np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx ** 2 + qy ** 2)],
    ], dtype=np.float64)
    return R


def numeric_key_for_filename(p: Path) -> Tuple[int, str]:
    """为补丁文件生成排序键：优先使用无扩展名的数字，否则用名称。"""
    stem = p.stem
    try:
        return int(stem), ''
    except ValueError:
        return (2**31 - 1), stem


def transform_points(points: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (points @ R.T) + t.reshape(1, 3)


def load_point_cloud_points(path: Path) -> np.ndarray:
    pc = o3d.io.read_point_cloud(str(path))
    return np.asarray(pc.points, dtype=np.float64)


def save_point_cloud_points(path: Path, points: np.ndarray):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    o3d.io.write_point_cloud(str(path), pc, write_ascii=False, compressed=False)


def reconstruct(patches_dir: Path, poses_path: Path, output_path: Path, downsample_voxel: float = 0.0):
    if not patches_dir.exists() or not patches_dir.is_dir():
        raise FileNotFoundError(f'patches 目录不存在: {patches_dir}')
    if not poses_path.exists():
        raise FileNotFoundError(f'位姿文件不存在: {poses_path}')

    name_to_pose, list_poses = parse_poses_file(poses_path)

    patch_files = sorted([p for p in patches_dir.iterdir() if p.suffix.lower() in ('.pcd', '.ply')],
                         key=numeric_key_for_filename)
    if not patch_files:
        raise RuntimeError('未在 patches 目录中找到 .pcd/.ply 文件')

    # 预统计
    total_points = 0
    transformed_chunks: List[np.ndarray] = []

    if name_to_pose:
        # 使用名字匹配
        missing = 0
        for pf in patch_files:
            key = pf.name
            if key not in name_to_pose:
                missing += 1
                continue
            t, q = name_to_pose[key]
            R = quaternion_to_rotation_matrix(q)
            pts = load_point_cloud_points(pf)
            if pts.size == 0:
                continue
            pts_w = transform_points(pts, R, t)
            transformed_chunks.append(pts_w)
            total_points += pts_w.shape[0]
        if missing:
            print(f'⚠️  有 {missing} 个patch在位姿文件中未找到对应条目（将被跳过）')
    else:
        # 按序对齐
        if len(list_poses) < len(patch_files):
            print(f'⚠️  位姿数量({len(list_poses)})少于patch数量({len(patch_files)}), 仅按最小数量对齐')
        n = min(len(list_poses), len(patch_files))
        for i in range(n):
            t, q = list_poses[i]
            R = quaternion_to_rotation_matrix(q)
            pts = load_point_cloud_points(patch_files[i])
            if pts.size == 0:
                continue
            pts_w = transform_points(pts, R, t)
            transformed_chunks.append(pts_w)
            total_points += pts_w.shape[0]

    if not transformed_chunks:
        raise RuntimeError('没有有效的点可用于重建，请检查 patches 与 poses 对齐关系')

    all_points = np.vstack(transformed_chunks)
    print(f'✅ 重建完成，累计点数: {all_points.shape[0]}（拼接前总计 {total_points}）')

    # 可选体素下采样
    if downsample_voxel and downsample_voxel > 0.0:
        print(f'🔧 体素下采样: voxel={downsample_voxel} m')
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(all_points)
        pc = pc.voxel_down_sample(voxel_size=float(downsample_voxel))
        all_points = np.asarray(pc.points)
        print(f'   下采样后点数: {all_points.shape[0]}')

    save_point_cloud_points(output_path, all_points)
    print(f'💾 已保存重建地图: {output_path}')


def main():
    parser = argparse.ArgumentParser(description='使用 patches + poses 离线重建整图')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--pgo-dir', type=str, help='PGO 保存目录（包含 patches/ 和 poses.txt）')
    group.add_argument('--patches-dir', type=str, help='直接指定 patches 目录')
    parser.add_argument('--poses', type=str, default=None, help='位姿文件路径（默认：<pgo-dir>/poses.txt）')
    parser.add_argument('--output', type=str, required=True, help='输出 PCD 路径')
    parser.add_argument('--downsample-voxel', type=float, default=0.0, help='体素下采样（米），0=不下采样')

    args = parser.parse_args()

    if args.pgo_dir:
        pgo_dir = Path(args.pgo_dir).resolve()
        patches_dir = pgo_dir / 'patches'
        poses_path = Path(args.poses).resolve() if args.poses else (pgo_dir / 'poses.txt')
    else:
        patches_dir = Path(args.patches_dir).resolve()
        poses_path = Path(args.poses).resolve() if args.poses else None
        if poses_path is None:
            # 尝试在父目录寻找 poses.txt
            candidate = patches_dir.parent / 'poses.txt'
            if candidate.exists():
                poses_path = candidate
            else:
                raise FileNotFoundError('未指定 --poses，且无法在 patches 父目录找到 poses.txt')

    output_path = Path(args.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    reconstruct(patches_dir, poses_path, output_path, downsample_voxel=float(args.downsample_voxel))


if __name__ == '__main__':
    main()

