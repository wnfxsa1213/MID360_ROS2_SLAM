#!/usr/bin/env python3
"""
修复地图精细化工具中的copy()方法调用问题
Open3D点云对象没有copy()方法，需要使用copy.deepcopy()
"""

import os
import re
from pathlib import Path

def fix_copy_methods(directory):
    """修复目录中所有Python文件的copy()方法调用"""
    pattern = r'(\w+)\.copy\(\)'
    replacement = r'copy.deepcopy(\1)'

    # 遍历所有Python文件
    for file_path in Path(directory).rglob('*.py'):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # 检查是否需要修复
            if '.copy()' in content:
                print(f"修复文件: {file_path}")

                # 检查是否已导入copy模块
                if 'import copy' not in content:
                    # 在import语句后添加copy导入
                    import_lines = []
                    other_lines = []
                    in_imports = True

                    for line in content.split('\n'):
                        if line.strip().startswith('import ') or line.strip().startswith('from '):
                            import_lines.append(line)
                        elif line.strip() == '' and in_imports:
                            import_lines.append(line)
                        else:
                            if in_imports and line.strip():
                                import_lines.append('import copy')
                                in_imports = False
                            other_lines.append(line)

                    if in_imports:  # 如果文件只有import语句
                        import_lines.append('import copy')

                    content = '\n'.join(import_lines + other_lines)

                # 替换.copy()调用
                # 特殊处理numpy数组的copy()调用（保持不变）
                content = re.sub(r'(\w+)\.copy\(\)', lambda m:
                    f'copy.deepcopy({m.group(1)})' if not m.group(1).endswith('_array') and not 'np.' in m.group(0) else m.group(0),
                    content)

                # 保持numpy数组的copy()不变
                content = re.sub(r'copy\.deepcopy\((\w*points\w*)\)', r'\1.copy()', content)
                content = re.sub(r'copy\.deepcopy\((\w*array\w*)\)', r'\1.copy()', content)
                content = re.sub(r'copy\.deepcopy\((self\.\w*points\w*)\)', r'\1.copy()', content)
                content = re.sub(r'copy\.deepcopy\((self\.\w*array\w*)\)', r'\1.copy()', content)
                content = re.sub(r'copy\.deepcopy\((\w*_points)\)', r'\1.copy()', content)
                content = re.sub(r'copy\.deepcopy\((current_transformation)\)', r'\1.copy()', content)

                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(content)

                print(f"  ✅ 修复完成")

        except Exception as e:
            print(f"❌ 修复文件 {file_path} 失败: {e}")

if __name__ == "__main__":
    map_refinement_dir = "/home/tianyu/codes/Mid360map/tools/map_refinement"
    print("开始修复地图精细化工具中的copy()方法调用...")
    fix_copy_methods(map_refinement_dir)
    print("修复完成！")