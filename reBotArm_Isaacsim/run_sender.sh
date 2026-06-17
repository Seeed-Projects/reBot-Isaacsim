#!/usr/bin/env bash
# 发送端启动脚本 / Sender launcher.
# 使用 reBotArm_control_py 的 uv 环境运行 gravity_joint_sender.py。
# Run gravity_joint_sender.py inside the reBotArm_control_py uv environment.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CONTROL_PY="${REPO_ROOT}/third_party/reBotArm_control_py"
UV_PYTHON="${CONTROL_PY}/.venv/bin/python"

if [[ ! -f "${UV_PYTHON}" ]]; then
  echo "[error] 未找到 reBotArm_control_py 的 uv Python: ${UV_PYTHON} / uv Python for reBotArm_control_py not found: ${UV_PYTHON}" >&2
  echo "[hint] 请先在 ${CONTROL_PY} 目录下运行 uv sync / please run 'uv sync' under ${CONTROL_PY} first" >&2
  exit 1
fi

exec "${UV_PYTHON}" "${SCRIPT_DIR}/gravity_joint_sender.py" "$@"
