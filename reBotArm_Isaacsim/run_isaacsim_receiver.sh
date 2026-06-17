#!/usr/bin/env bash
# Isaac Sim 接收端启动脚本 / Isaac Sim receiver launcher.
# 使用 Isaac 官方 python.sh 运行 isaacsim_joint_receiver.py。
# Run isaacsim_joint_receiver.py via the official Isaac Sim python.sh.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ISAACSIM_ROOT="${ISAACSIM_ROOT:-/home/seeed/IsaacSim/_build/linux-x86_64/release}"
ISAACSIM_PYTHON="${ISAACSIM_ROOT}/python.sh"

if [[ ! -f "${ISAACSIM_PYTHON}" ]]; then
  echo "[error] 未找到 Isaac Sim python.sh: ${ISAACSIM_PYTHON} / Isaac Sim python.sh not found: ${ISAACSIM_PYTHON}" >&2
  echo "[hint] 请设置 ISAACSIM_ROOT 环境变量指向 Isaac Sim 运行目录 / please set ISAACSIM_ROOT to your Isaac Sim runtime directory" >&2
  exit 1
fi

export PYTHONPATH="${REPO_ROOT}:${PYTHONPATH}"

exec bash "${ISAACSIM_PYTHON}" "${SCRIPT_DIR}/isaacsim_joint_receiver.py" "$@"
