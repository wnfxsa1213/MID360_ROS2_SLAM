#!/usr/bin/env python3
"""
PCD 文件索引与快速查询工具

- 构建/更新索引：递归扫描 root 下的 .pcd 文件，保存为 JSON 索引
- 查询：按文件名/模式查找或获取最新的 PCD 文件，避免每次全目录遍历

用法示例：
  python3 tools/pcd_index.py --root saved_maps --ensure --latest
  python3 tools/pcd_index.py --root saved_maps --ensure --find map.pcd
  python3 tools/pcd_index.py --root saved_maps --build
"""

import argparse
import fnmatch
import json
import os
import sys
import time
from pathlib import Path
from typing import List, Dict, Any


def scan_pcds(root: Path) -> List[Dict[str, Any]]:
    files = []
    for dirpath, _dirnames, filenames in os.walk(root):
        for name in filenames:
            if not name.lower().endswith('.pcd'):
                continue
            p = Path(dirpath) / name
            try:
                st = p.stat()
            except FileNotFoundError:
                continue
            files.append({
                'path': str(p.resolve()),
                'name': name,
                'mtime': st.st_mtime,
                'size': st.st_size,
            })
    return files


def load_index(index_path: Path) -> Dict[str, Any]:
    if not index_path.exists():
        return {}
    try:
        with open(index_path, 'r', encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return {}


def save_index(index_path: Path, root: Path, files: List[Dict[str, Any]]):
    data = {
        'version': 1,
        'root': str(root.resolve()),
        'generated_at': time.time(),
        'files': files,
    }
    index_path.parent.mkdir(parents=True, exist_ok=True)
    with open(index_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False)


def ensure_index(root: Path, index_path: Path, rebuild: bool = False) -> Dict[str, Any]:
    idx = load_index(index_path)
    if rebuild or not idx or Path(idx.get('root', '')) != root.resolve():
        files = scan_pcds(root)
        save_index(index_path, root, files)
        return load_index(index_path)
    return idx


def find_by_name(idx: Dict[str, Any], name: str) -> str:
    files = idx.get('files', [])
    # 精确匹配 basename
    exact = [f for f in files if f['name'] == name]
    if exact:
        return sorted(exact, key=lambda x: x['mtime'], reverse=True)[0]['path']
    # 模式匹配（如 *map*.pcd）
    pats = [f for f in files if fnmatch.fnmatch(f['name'], name)]
    if pats:
        return sorted(pats, key=lambda x: x['mtime'], reverse=True)[0]['path']
    # 退化：包含子串
    sub = [f for f in files if name in f['name']]
    if sub:
        return sorted(sub, key=lambda x: x['mtime'], reverse=True)[0]['path']
    return ''


def latest(idx: Dict[str, Any]) -> str:
    files = idx.get('files', [])
    if not files:
        return ''
    return sorted(files, key=lambda x: x['mtime'], reverse=True)[0]['path']


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', required=True, help='PCD 根目录（通常为 saved_maps）')
    ap.add_argument('--index', default='', help='索引文件路径（默认 root/.pcd_index.json）')
    ap.add_argument('--build', action='store_true', help='强制重建索引')
    ap.add_argument('--ensure', action='store_true', help='若索引不存在则构建')
    ap.add_argument('--find', default='', help='按文件名/通配符查询（返回最新匹配）')
    ap.add_argument('--latest', action='store_true', help='返回最新的 PCD')
    args = ap.parse_args()

    root = Path(args.root)
    if not root.exists():
        print(f"错误: 根目录不存在: {root}", file=sys.stderr)
        return 2
    index_path = Path(args.index) if args.index else (root / '.pcd_index.json')

    idx = {}
    if args.build:
        idx = ensure_index(root, index_path, rebuild=True)
    elif args.ensure or args.find or args.latest:
        idx = ensure_index(root, index_path, rebuild=not index_path.exists())
    else:
        idx = load_index(index_path)
        if not idx:
            idx = ensure_index(root, index_path, rebuild=True)

    if args.find:
        out = find_by_name(idx, args.find)
        if not out:
            return 1
        print(out)
        return 0
    if args.latest:
        out = latest(idx)
        if not out:
            return 1
        print(out)
        return 0

    # 默认行为：什么也不做，仅构建
    print(f"已索引 {len(idx.get('files', []))} 个 .pcd 文件 到 {index_path}")
    return 0


if __name__ == '__main__':
    sys.exit(main())

