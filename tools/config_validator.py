#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置文件验证和锁定机制
防止意外修改生成的配置文件
"""

import os
import sys
import hashlib
import json
import yaml
from datetime import datetime
from pathlib import Path

class ConfigValidator:
    """配置验证器和锁定机制"""

    def __init__(self, project_root: str):
        self.project_root = Path(project_root)
        self.lock_file = self.project_root / "config" / ".config_lock.json"

    def generate_file_hash(self, file_path: Path) -> str:
        """生成文件的MD5哈希值"""
        try:
            with open(file_path, 'rb') as f:
                return hashlib.md5(f.read()).hexdigest()
        except:
            return ""

    def create_config_lock(self) -> bool:
        """创建配置锁定文件"""
        try:
            config_files = [
                "ws_livox/src/fastlio2/config/lio.yaml",
                "ws_livox/src/livox_ros_driver2/config/MID360_config.json",
                "config/launch_config.yaml",
                "ws_livox/src/pgo/config/pgo.yaml",
                "ws_livox/src/hba/config/hba.yaml",
                "ws_livox/src/localizer/config/localizer.yaml"
            ]

            lock_data = {
                "created_at": datetime.now().isoformat(),
                "files": {}
            }

            for config_file in config_files:
                file_path = self.project_root / config_file
                if file_path.exists():
                    lock_data["files"][config_file] = {
                        "hash": self.generate_file_hash(file_path),
                        "size": file_path.stat().st_size,
                        "modified": datetime.fromtimestamp(file_path.stat().st_mtime).isoformat()
                    }

            # 确保锁定文件目录存在
            self.lock_file.parent.mkdir(parents=True, exist_ok=True)

            with open(self.lock_file, 'w', encoding='utf-8') as f:
                json.dump(lock_data, f, indent=2, ensure_ascii=False)

            print(f"✓ 配置锁定文件已创建: {self.lock_file}")
            return True

        except Exception as e:
            print(f"✗ 创建配置锁定文件失败: {e}")
            return False

    def verify_config_integrity(self) -> dict:
        """验证配置文件完整性"""
        if not self.lock_file.exists():
            return {"status": "no_lock", "message": "未找到配置锁定文件"}

        try:
            with open(self.lock_file, 'r', encoding='utf-8') as f:
                lock_data = json.load(f)

            verification_result = {
                "status": "verified",
                "changed_files": [],
                "missing_files": [],
                "details": {}
            }

            for file_path, expected_data in lock_data["files"].items():
                full_path = self.project_root / file_path

                if not full_path.exists():
                    verification_result["missing_files"].append(file_path)
                    verification_result["status"] = "error"
                    continue

                current_hash = self.generate_file_hash(full_path)
                expected_hash = expected_data["hash"]

                if current_hash != expected_hash:
                    verification_result["changed_files"].append(file_path)
                    verification_result["status"] = "modified"

                verification_result["details"][file_path] = {
                    "expected_hash": expected_hash,
                    "current_hash": current_hash,
                    "match": current_hash == expected_hash
                }

            return verification_result

        except Exception as e:
            return {"status": "error", "message": f"验证失败: {e}"}

    def print_verification_report(self, result: dict):
        """打印验证报告"""
        print("="*60)
        print("📋 配置文件完整性验证报告")
        print("="*60)

        if result["status"] == "no_lock":
            print("⚠️  未找到配置锁定文件")
            print("   建议运行: python tools/config_validator.py --create-lock")
            return

        if result["status"] == "error":
            print(f"❌ 验证过程中出现错误: {result.get('message', '未知错误')}")
            if result.get("missing_files"):
                print("\n缺失的配置文件:")
                for file in result["missing_files"]:
                    print(f"   - {file}")
            return

        if result["status"] == "verified":
            print("✅ 所有配置文件完整性验证通过")
        elif result["status"] == "modified":
            print("⚠️  检测到配置文件被修改:")
            for file in result["changed_files"]:
                print(f"   - {file}")
            print("\n建议:")
            print("   1. 如果是有意修改，请重新生成配置:")
            print("      python tools/config_generator.py")
            print("   2. 如果是意外修改，请从备份恢复:")
            print("      ls config/backups/")

        print(f"\n总计检查文件: {len(result['details'])}")
        print(f"完整性验证: {'通过' if result['status'] == 'verified' else '失败'}")

def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='配置文件验证和锁定工具')
    parser.add_argument('--create-lock', action='store_true',
                        help='创建配置锁定文件')
    parser.add_argument('--verify', action='store_true',
                        help='验证配置文件完整性')
    parser.add_argument('--project-root',
                        default=os.path.dirname(os.path.dirname(__file__)),
                        help='项目根目录')

    args = parser.parse_args()

    validator = ConfigValidator(args.project_root)

    if args.create_lock:
        validator.create_config_lock()
    elif args.verify:
        result = validator.verify_config_integrity()
        validator.print_verification_report(result)
    else:
        # 默认执行验证
        result = validator.verify_config_integrity()
        validator.print_verification_report(result)

if __name__ == "__main__":
    main()