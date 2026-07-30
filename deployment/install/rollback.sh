#!/usr/bin/env bash
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "rollback must run as root" >&2
  exit 1
fi

exec /usr/local/sbin/cleanbot-updater \
  --config /etc/cleanbot/updater.json rollback
