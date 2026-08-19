#!/usr/bin/env bash
# 文件作用：安装首个 Cleanbot 发布目录、升级器、服务配置和签名公钥。
set -euo pipefail

# 方法作用：输出安装脚本参数格式并以参数错误状态退出。
usage() {
  echo "usage: $0 RELEASE_DIRECTORY ED25519_PUBLIC_KEY [--start]" >&2
  exit 2
}

if [[ ${EUID} -ne 0 ]]; then
  echo "install must run as root" >&2
  exit 1
fi
if [[ $# -lt 2 || $# -gt 3 ]]; then
  usage
fi

release_source=$(readlink -f -- "$1")
public_key=$(readlink -f -- "$2")
start_services=false
if [[ $# -eq 3 ]]; then
  [[ $3 == "--start" ]] || usage
  start_services=true
fi
[[ -d "${release_source}/install" ]] || usage
[[ -f "${release_source}/install/setup.bash" ]] || usage
[[ -f "${release_source}/VERSION" ]] || usage
[[ -f "${public_key}" ]] || usage

version=$(tr -d '\r\n' < "${release_source}/VERSION")
if [[ ! ${version} =~ ^v?(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)$ ]]; then
  echo "release VERSION must be stable semantic version" >&2
  exit 1
fi

script_directory=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
deployment_root=$(readlink -f -- "${script_directory}/..")
release_target="/opt/cleanbot/releases/${version}"
reuse_release=false
if [[ -e ${release_target} || -L ${release_target} ]]; then
  if [[ -L ${release_target} || ! -d ${release_target} ||
        ! -f ${release_target}/VERSION ||
        ! -f ${release_target}/install/setup.bash ||
        $(tr -d '\r\n' < "${release_target}/VERSION") != "${version}" ]]; then
    echo "existing release is incomplete or unsafe: ${version}" >&2
    exit 1
  fi
  reuse_release=true
fi

if ! id -u cleanbot >/dev/null 2>&1; then
  useradd --system --home-dir /var/lib/cleanbot --shell /usr/sbin/nologin \
    --groups dialout cleanbot
fi

install -d -m 0755 /opt/cleanbot/releases
install -d -m 0755 /etc/cleanbot /usr/local/lib/cleanbot
install -d -m 0755 /usr/local/libexec /usr/local/sbin
install -d -o cleanbot -g cleanbot -m 0750 /var/lib/cleanbot
install -d -o cleanbot -g cleanbot -m 0750 /var/lib/cleanbot/runtime
install -d -o cleanbot -g cleanbot -m 0750 \
  /var/lib/cleanbot/runtime/updater-ros-home
install -d -o cleanbot -g cleanbot -m 0750 \
  /var/lib/cleanbot/runtime/updater-ros-log
install -d -o root -g root -m 0700 /var/lib/cleanbot/update/staging
install -d -o cleanbot -g cleanbot -m 0750 /var/log/cleanbot/ros

temporary_release=""
temporary_link="/opt/cleanbot/.current.new.$$"
# 方法作用：脚本异常退出时删除尚未完成的临时安装目录。
cleanup() {
  if [[ -n ${temporary_release} && -d ${temporary_release} ]]; then
    rm -rf -- "${temporary_release}"
  fi
  rm -f -- "${temporary_link}"
}
trap cleanup EXIT
if [[ ${reuse_release} == false ]]; then
  temporary_release=$(mktemp -d "/opt/cleanbot/releases/.${version}.install.XXXXXX")
  cp -a -- "${release_source}/." "${temporary_release}/"
  chown -R root:root "${temporary_release}"
  chmod 0755 "${temporary_release}"
  mv -- "${temporary_release}" "${release_target}"
  temporary_release=""
fi

maintenance_initializer="${release_target}/install/cleanbot_mission/lib/cleanbot_mission/maintenance_store_init"
if [[ ! -x ${maintenance_initializer} ]]; then
  echo "maintenance store initializer is missing from release" >&2
  exit 1
fi
runuser -u cleanbot -- "${maintenance_initializer}" "/var/lib/cleanbot/runtime/maintenance.lock"

install -m 0755 "${deployment_root}/bin/cleanbot-start" \
  /usr/local/libexec/cleanbot-start
install -m 0755 "${deployment_root}/bin/cleanbot-updater" \
  /usr/local/sbin/cleanbot-updater
install -m 0755 "${deployment_root}/bin/cleanbot-ros-cli" \
  /usr/local/libexec/cleanbot-ros-cli
install -m 0755 "${deployment_root}/install/rollback.sh" \
  /usr/local/sbin/cleanbot-rollback
install -m 0755 "${deployment_root}/updater/cleanbot_updater.py" \
  /usr/local/lib/cleanbot/cleanbot_updater.py
install -m 0644 "${public_key}" /etc/cleanbot/update-public.pem
if [[ ! -e /etc/cleanbot/updater.json ]]; then
  install -m 0644 "${deployment_root}/config/updater.json" \
    /etc/cleanbot/updater.json
fi
install -m 0644 "${deployment_root}/systemd/cleanbot.service" \
  /etc/systemd/system/cleanbot.service
install -m 0644 "${deployment_root}/systemd/cleanbot-update.service" \
  /etc/systemd/system/cleanbot-update.service
install -m 0644 "${deployment_root}/systemd/cleanbot-update.timer" \
  /etc/systemd/system/cleanbot-update.timer
install -m 0644 "${deployment_root}/udev/99-cleanbot.rules" \
  /etc/udev/rules.d/99-cleanbot.rules
install -m 0644 "${deployment_root}/logrotate/cleanbot" \
  /etc/logrotate.d/cleanbot

state_path=/var/lib/cleanbot/update/state.json
if [[ ! -e ${state_path} ]]; then
  printf '{"currentRelease":"%s","lastError":"","phase":"IDLE","previousRelease":"","schemaVersion":1,"stagedRelease":""}\n' \
    "${version}" > "${state_path}"
  chmod 0600 "${state_path}"
fi

systemctl daemon-reload
udevadm control --reload-rules
systemctl enable cleanbot.service cleanbot-update.service cleanbot-update.timer

# The release becomes active only after all persistent support files and
# service definitions have been installed successfully.
ln -s -- "${release_target}" "${temporary_link}"
mv -Tf -- "${temporary_link}" /opt/cleanbot/current

if [[ ${start_services} == true ]]; then
  systemctl start cleanbot.service cleanbot-update.service cleanbot-update.timer
fi

trap - EXIT
echo "installed Cleanbot ${version}; configure serial devices before starting"
