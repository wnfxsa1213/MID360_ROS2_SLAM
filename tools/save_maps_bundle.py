#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
一键保存 SLAM 地图与轨迹（LIO / PGO / HBA）

功能
- 保存 LIO 全局累积地图 + LIO 轨迹
- 保存 PGO 优化后的地图（目录：map.pcd/poses.txt/patches/）
- 保存 HBA 优化后的轨迹（仅轨迹）

用法示例
  python3 tools/save_maps_bundle.py \
    --output-dir saved_maps \
    --name run1 \
    --format pcd \
    --pgo-save-patches

注意
- PGO 的保存服务固定输出 map.pcd（不支持切换为 ply）。
- 需确保对应节点在运行：fastlio2/pgo/hba。
"""

import os
import sys
import argparse
import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node

from interface.srv import SaveMaps, SavePoses


class BundleMapSaver(Node):
    def __init__(self, want_lio: bool, want_pgo: bool, want_hba: bool):
        super().__init__('bundle_map_saver')

        self.want_lio = want_lio
        self.want_pgo = want_pgo
        self.want_hba = want_hba

        self.save_maps_fastlio2 = None
        self.save_poses_fastlio2 = None
        self.save_maps_pgo = None
        self.save_poses_hba = None

        if self.want_lio:
            self.save_maps_fastlio2 = self.create_client(SaveMaps, 'fastlio2/save_maps')
            self.save_poses_fastlio2 = self.create_client(SavePoses, 'fastlio2/save_poses')

        if self.want_pgo:
            self.save_maps_pgo = self.create_client(SaveMaps, 'pgo/save_maps')

        if self.want_hba:
            self.save_poses_hba = self.create_client(SavePoses, 'hba/save_poses')

    def wait_services(self, timeout_sec: float = 10.0) -> bool:
        ok = True
        if self.want_lio:
            self.get_logger().info('等待 LIO 服务可用...')
            if not self.save_maps_fastlio2.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error('fastlio2/save_maps 不可用')
                ok = False
            if not self.save_poses_fastlio2.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error('fastlio2/save_poses 不可用')
                ok = False

        if self.want_pgo:
            self.get_logger().info('等待 PGO 服务可用...')
            if not self.save_maps_pgo.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error('pgo/save_maps 不可用')
                ok = False

        if self.want_hba:
            self.get_logger().info('等待 HBA 服务可用...')
            if not self.save_poses_hba.wait_for_service(timeout_sec=timeout_sec):
                self.get_logger().error('hba/save_poses 不可用')
                ok = False

        return ok

    def call_save_maps(self, client, file_path: str, save_patches: bool = False) -> bool:
        req = SaveMaps.Request()
        req.file_path = file_path
        req.save_patches = save_patches
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.success:
            self.get_logger().info(f'✅ 地图保存成功: {res.message}')
            return True
        msg = res.message if res else '服务调用失败'
        self.get_logger().error(f'❌ 地图保存失败: {msg}')
        return False

    def call_save_poses(self, client, file_path: str) -> bool:
        req = SavePoses.Request()
        req.file_path = file_path
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res and res.success:
            self.get_logger().info(f'✅ 轨迹保存成功: {res.message}')
            return True
        msg = res.message if res else '服务调用失败'
        self.get_logger().error(f'❌ 轨迹保存失败: {msg}')
        return False


def main():
    parser = argparse.ArgumentParser(description='一键保存 LIO/PGO/HBA 地图与轨迹')
    parser.add_argument('--output-dir', '-o', type=str, default='./saved_maps',
                        help='输出目录（默认: ./saved_maps）')
    parser.add_argument('--name', '-n', type=str, default=None,
                        help='命名前缀；默认使用时间戳')
    parser.add_argument('--format', '-f', choices=['pcd', 'ply'], default='pcd',
                        help='LIO地图保存格式（PGO 固定输出 map.pcd）')

    parser.add_argument('--no-lio', action='store_true', help='不保存 LIO 地图与轨迹')
    parser.add_argument('--no-pgo', action='store_true', help='不保存 PGO 地图')
    parser.add_argument('--no-hba', action='store_true', help='不保存 HBA 轨迹')
    parser.add_argument('--hba-refine', action='store_true', default=False,
                        help='在保存PGO后，自动调用 HBA refine_map 并保存 HBA 轨迹')
    parser.add_argument('--hba-wait-seconds', type=int, default=30,
                        help='HBA 优化等待秒数 (默认: 30)')

    parser.add_argument('--pgo-save-patches', action='store_true', default=True,
                        help='PGO 同时导出 patches（默认启用）')
    parser.add_argument('--no-pgo-save-patches', action='store_true',
                        help='禁用 PGO patches 导出')

    args = parser.parse_args()

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.name:
        base_name = args.name
    else:
        base_name = datetime.datetime.now().strftime('mid360_%Y%m%d_%H%M%S')

    want_lio = not args.no_lio
    want_pgo = not args.no_pgo
    want_hba = not args.no_hba

    # PGO patches flag
    pgo_save_patches = True
    if args.no_pgo_save_patches:
        pgo_save_patches = False
    elif args.pgo_save_patches:
        pgo_save_patches = True

    # 生成路径
    lio_map_path = output_dir / f'{base_name}_lio_map.{args.format}'
    lio_traj_path = output_dir / f'{base_name}_lio_trajectory.txt'
    pgo_dir = output_dir / f'{base_name}_pgo'
    hba_traj_path = output_dir / f'{base_name}_hba_poses.txt'

    rclpy.init()
    try:
        node = BundleMapSaver(want_lio=want_lio, want_pgo=want_pgo, want_hba=want_hba)
        if not node.wait_services():
            print('❌ 所需服务不可用，终止。')
            sys.exit(1)

        total = 0
        success = 0

        # LIO 地图与轨迹
        if want_lio:
            total += 1
            node.get_logger().info(f'保存 LIO 地图到: {lio_map_path}')
            if node.call_save_maps(node.save_maps_fastlio2, str(lio_map_path)):
                success += 1

            total += 1
            node.get_logger().info(f'保存 LIO 轨迹到: {lio_traj_path}')
            if node.call_save_poses(node.save_poses_fastlio2, str(lio_traj_path)):
                success += 1

        # PGO 地图（优化后）
        if want_pgo:
            pgo_dir.mkdir(parents=True, exist_ok=True)
            total += 1
            node.get_logger().info(f'保存 PGO 地图到目录: {pgo_dir} (map.pcd/poses.txt/patches)')
            if node.call_save_maps(node.save_maps_pgo, str(pgo_dir), save_patches=pgo_save_patches):
                success += 1
                if args.hba_refine and want_hba:
                    # 调用 HBA refine_map -> 等待 -> 保存 HBA 轨迹
                    try:
                        from interface.srv import RefineMap
                    except Exception:
                        node.get_logger().warn('未找到 RefineMap 服务接口，跳过 HBA refine')
                    else:
                        refine_client = node.create_client(RefineMap, 'hba/refine_map')
                        if not refine_client.wait_for_service(timeout_sec=10.0):
                            node.get_logger().warn('hba/refine_map 不可用，跳过 HBA refine')
                        else:
                            node.get_logger().info(f'调用 HBA refine_map, maps_path={pgo_dir}')
                            req = RefineMap.Request()
                            req.maps_path = str(pgo_dir)
                            fut = refine_client.call_async(req)
                            rclpy.spin_until_future_complete(node, fut)
                            if fut.result() and fut.result().success:
                                import time
                                wait_s = max(5, int(args.hba_wait_seconds))
                                node.get_logger().info(f'⏳ HBA优化进行中，等待 {wait_s}s 后保存轨迹...')
                                time.sleep(wait_s)
                                total += 1
                                node.get_logger().info(f'保存 HBA 轨迹到: {hba_traj_path}')
                                if node.call_save_poses(node.save_poses_hba, str(hba_traj_path)):
                                    success += 1
                                else:
                                    node.get_logger().error('HBA 轨迹保存失败（可能优化尚未完成）')
                            else:
                                node.get_logger().warn('hba/refine_map 调用失败，跳过 HBA 轨迹保存')

        # HBA 轨迹
        if want_hba and not args.hba_refine:
            total += 1
            node.get_logger().info(f'保存 HBA 轨迹到: {hba_traj_path}')
            if node.call_save_poses(node.save_poses_hba, str(hba_traj_path)):
                success += 1

        print(f'\n📊 保存完成: {success}/{total} 项成功')
        if success == total:
            print(f'✅ 输出目录: {output_dir}')
            if want_lio:
                print(f'  • LIO 地图: {lio_map_path.name}')
                print(f'  • LIO 轨迹: {lio_traj_path.name}')
            if want_pgo:
                print(f'  • PGO 目录: {pgo_dir.name} (含 map.pcd/poses.txt/patches)')
            if want_hba:
                print(f'  • HBA 轨迹: {hba_traj_path.name}')
        else:
            sys.exit(1)

    except KeyboardInterrupt:
        print('\n⚠️ 已取消')
    except Exception as e:
        print(f'❌ 错误: {e}')
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
