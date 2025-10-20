#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

missing=0

if ! grep -q "文档更新机制" "${REPO_ROOT}/README.md"; then
  echo "[docs] README.md 缺少文档更新机制章节" >&2
  missing=1
fi

if [ ! -f "${REPO_ROOT}/docs/documentation_workflow.md" ]; then
  echo "[docs] 未找到 docs/documentation_workflow.md" >&2
  missing=1
fi

if [ ${missing} -ne 0 ]; then
  echo "文档检查失败" >&2
  exit 1
fi

echo "文档检查通过"
