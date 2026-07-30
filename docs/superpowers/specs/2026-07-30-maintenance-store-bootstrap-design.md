# 首次部署维护状态初始化设计

日期：2026-07-30

## 背景与根因

`mission_manager_node` 启动时只读取
`/var/lib/cleanbot/runtime/maintenance.lock`。记录缺失、损坏或无法读取时，
节点按 fail-closed 原则关闭任务 Action 准入，不会自行创建记录。

当前安装脚本创建了维护状态目录，但没有创建初始记录。因此，全新安装后
任务节点能够启动，却会一直拒绝任务。升级既有系统时不一定暴露该问题，
因为原系统可能已经存在记录。

## 目标

- 首次安装时建立由现有 `MaintenanceStore` 生成的可信初始记录。
- 已有有效记录必须原样保留。
- 已有损坏、类型错误或不可访问的记录必须导致安装失败，禁止静默覆盖。
- 任务节点继续只负责恢复记录，保持现有 fail-closed 安全边界。
- 初始化操作以运行服务的 `cleanbot` 用户执行，确保文件所有权一致。

## 方案

在 `cleanbot_mission` 包中增加一个小型命令行程序
`maintenance_store_init`。程序接收且只接收一个状态文件路径，构造现有
`MaintenanceStore`，调用 `initializeGenesis()`，并根据返回状态输出明确
结果和退出码。

安装脚本在以下条件全部成立后调用该程序：

1. `cleanbot` 用户及 `/var/lib/cleanbot/runtime` 已创建；
2. 新发布目录已经完整复制到 `/opt/cleanbot/releases/<version>`；
3. 初始化程序存在于该发布的 ROS2 安装树中。

调用通过 `runuser -u cleanbot -- ...` 执行，目标固定为
`/var/lib/cleanbot/runtime/maintenance.lock`。初始化成功后，安装脚本才继续
安装 systemd 文件、切换 `/opt/cleanbot/current` 和按需启动服务。

## 数据流

```text
install.sh
  -> 创建 cleanbot 用户和 runtime 目录
  -> 安装候选 release
  -> 以 cleanbot 用户运行 maintenance_store_init
       -> MaintenanceStore::initializeGenesis()
          -> 记录缺失：原子创建 genesis
          -> 记录有效：保持原记录并成功返回
          -> 记录损坏或 I/O 异常：拒绝并失败返回
  -> 安装系统文件并切换 current
```

## 错误处理

- 参数数量不正确：打印用法并返回非零。
- 初始化程序路径缺失或不可执行：安装脚本立即失败。
- 状态记录损坏、目录不安全、权限错误或落盘失败：工具打印
  `MaintenanceStore` 返回的错误，安装脚本因 `set -e` 停止。
- 已存在有效记录：视为幂等成功，不递增 generation，不改变 inhibitor。
- 安装失败发生在 `current` 切换前，因此不会激活半完成的新版本。

## 测试与验证

测试按以下顺序执行：

1. 先增加部署契约测试，要求 CMake 安装初始化工具，且安装脚本必须以
   `cleanbot` 用户、固定路径调用它；确认测试在实现前失败。
2. 增加或复用 C++ 测试验证：缺失记录创建成功、重复初始化幂等、损坏记录
   不被覆盖。
3. 实现最小工具和安装脚本调用，使新增测试通过。
4. 在 Windows 执行全部 Python 契约测试。
5. 在树莓派隔离目录重新执行 ARM64 `colcon build`、`colcon test` 和
   `colcon test-result --verbose`。
6. 使用隔离状态和数据库完成静止安全联调：节点启动、QoS 晚订阅、
   Service、HTTP，以及 Action 接受、拒绝和无 RTK 超时。

## 非目标

- 不允许任务节点在运行时自动修复或覆盖维护记录。
- 不改变维护状态文件格式、generation 语义或升级安全门。
- 不安装或启动正式 systemd 服务，不覆盖树莓派现有工程目录。
- 不启动下位机串口、RTK 网络或任何可能驱动车辆的节点。
