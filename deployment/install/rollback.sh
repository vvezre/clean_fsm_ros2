#!/usr/bin/env bash
# 文件作用：调用已安装升级器，将系统紧急回滚到上一个可用版本。
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "rollback must run as root" >&2
  exit 1
fi

exec /usr/local/sbin/cleanbot-updater \
  --config /etc/cleanbot/updater.json rollback
