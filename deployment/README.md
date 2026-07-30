# Cleanbot 原生自动更新

生产运行使用 systemd，不在树莓派上引入 Docker 或 K8s。稳定版本由人工触发
GitHub Actions 的 `workflow_dispatch`；GitHub ARM64 Runner 编译、测试、打包并签名，
树莓派通过 WiFi 主动读取公开 GitHub Release。

## 发布密钥

在离线可信电脑生成 Ed25519 私钥和公钥：

```bash
openssl genpkey -algorithm ED25519 -out cleanbot-release-private.pem
openssl pkey -in cleanbot-release-private.pem -pubout -out cleanbot-release-public.pem
base64 -w0 cleanbot-release-private.pem
```

把最后一条命令的输出保存为受保护环境 `cleanbot-release` 中的 GitHub Actions Secret
`CLEANBOT_RELEASE_SIGNING_KEY_B64`。私钥不得复制到树莓派；首次安装只传公钥。
发布环境建议在 GitHub 中配置人工审批。

## 首次部署

Windows 必须已配置 SSH 密钥，能够用 `BatchMode` 登录树莓派。在 ARM64 Humble 环境
构建一个包含 `VERSION` 和 `install/setup.bash` 的初始发布目录，然后执行：

```powershell
.\deployment\windows\bootstrap-pi.ps1 `
  -PiHost 192.168.0.169 `
  -UserName ubuntu `
  -ReleaseDirectory C:\path\to\release `
  -PublicKey C:\path\to\cleanbot-release-public.pem
```

安装脚本默认安装并启用服务，但不立即启动机器人。配置实际串口序列号和 ROS 参数后：

```bash
sudo systemctl start cleanbot.service cleanbot-update.timer
systemctl status cleanbot.service
systemctl status cleanbot-update.timer
```

`deployment/udev/99-cleanbot.rules` 中的两个 `REPLACE_..._SERIAL` 必须替换为真机
`ID_SERIAL_SHORT`；未替换时规则不会匹配任何设备。

## 发布稳定版本

在 GitHub Actions 页面人工运行 `ARM64 stable release`，输入严格递增的稳定语义版本，
例如 `v1.2.3`。工作流先在无签名密钥的构建任务中执行部署契约测试、`colcon build`、
`colcon test` 和测试结果检查，再将构建产物交给受保护的签名发布任务。签名任务不检出、
不执行仓库代码，只验证产物后签名 `manifest.json` 并创建 GitHub Release。

树莓派每 30 分钟运行一次 `cleanbot-update.timer`。默认配置：

```json
{"autoDownload":true,"autoApply":false}
```

因此默认自动下载、验签和暂存，但不会自动停车切换版本。完成真车维护模式、停车和回滚
验证后，将 `/etc/cleanbot/updater.json` 的 `autoApply` 改为 `true`，更新器才会自动
申请维护模式、等待安全就绪、停止服务、切换版本、健康检查，并在失败时恢复旧版本。

自动更新拒绝低于或等于当前版本的 GitHub Release；人工回滚仍允许切回上一版本。成功
更新后只保留当前版本和上一版本，并清理已验证的暂存包，避免长期占满磁盘。下载、解压
和安装前还会保留固定磁盘余量。

若更新过程中断电，下一次自动更新会识别 `APPLYING`、`HEALTH_CHECK` 或
`ROLLING_BACK` 状态，重新进入维护模式并恢复到已知版本。恢复成功仍记录为 `FAILED`，
要求操作员确认后再继续发布。

常用命令：

```bash
sudo cleanbot-updater --config /etc/cleanbot/updater.json status
sudo cleanbot-updater --config /etc/cleanbot/updater.json check
sudo cleanbot-updater --config /etc/cleanbot/updater.json stage
sudo cleanbot-updater --config /etc/cleanbot/updater.json apply
sudo cleanbot-updater --config /etc/cleanbot/updater.json rollback
```

## 远程回滚与离线故障

Windows 上的辅助命令：

```powershell
.\deployment\windows\emergency-rollback.ps1 `
  -PiHost 192.168.0.169 `
  -UserName ubuntu
```

该脚本只是通过 SSH 调用正常回滚流程，仍要求任务节点可响应维护模式，绝不会绕过停车
安全边界。如果新版本连任务节点都无法启动，不能使用该脚本强行切换；应先现场断开驱动
使能或按下急停，确认车辆物理静止后，再由维护人员通过 SSH/控制台执行离线修复。
