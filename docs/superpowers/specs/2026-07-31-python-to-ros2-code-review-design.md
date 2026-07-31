# Python→ROS2 已完成功能代码评审设计

## 1. 评审目标

本次评审只讲已经完成的 Python→ROS2 迁移内容，重点是让评审者能够从源码中追踪一条任务的完整执行链，并理解每个节点的职责边界、接口选择、状态流转和安全处理。

本次不讨论部署、持续集成、发布包、远程升级、systemd、udev、logrotate 或后续未实现功能。

## 2. 评审输出

评审材料由以下部分组成：

1. Python 功能到 ROS2 节点的迁移对照表。
2. 节点职责和边界图。
3. Topic、Service、Action 接口矩阵。
4. 一条清扫任务的时序图和源码追踪表。
5. 异常、安全、暂停、取消、RTK 恢复和断点恢复流程图。
6. 节点入口方法、关键处理方法和辅助算法的代码索引。
7. 关键方法的统一注释说明。
8. 已有测试和静止安全联调证据索引。

## 3. 迁移范围

### 3.1 已纳入评审

- 配置读取、校验、SQLite 持久化和配置就绪状态。
- 下位机串口、协议编解码、ACK、状态发布、命令生命周期和重连安全处理。
- RTK 串口、NMEA 解析、航向同步、车辆中心点换算、NTRIP/RTCM3 和定位有效性。
- 纠偏目标接收、位置误差计算、直线控制和运动命令发布。
- 命令来源仲裁、命令租约、急停/安全门和唯一最终输出。
- 清扫、多路点、返航、暂停、取消、RTK 恢复和任务段级断点恢复。
- 打点、区域识别、覆盖路径生成、计划保存和计划提交。
- HTTP 请求校验、ROS2 接口调用、人工控制/急停和车辆状态返回。

### 3.2 本次不纳入

- 充电桩自动对接。
- 低电量自动返航策略。
- 当前任务段内部的面积级断点恢复和实际覆盖面积统计。
- 多路点任务的持久化断点恢复（如果代码只有清扫/返航检查点，则只讲已有部分）。
- 任务历史、报警历史和统计数据库。
- 部署、发布和远程升级链路。

## 4. 节点职责和源码索引

行号以当前源码快照为参考；评审讲解和文档索引同时记录方法名，避免补充注释后行号变化造成索引失效。

### 4.1 配置节点

职责：配置定义和校验、SQLite 读写、就绪状态发布、配置服务。

入口和关键方法：

- `src/cleanbot_config/src/config_manager_node.cpp`
  - `ConfigManagerNode()`：ROS2 publisher/service 创建和数据库初始化。
  - `evaluate_configuration()`：必填配置、默认值和类型校验。
  - `on_get()`：`/config/get` 服务。
  - `on_set()`：`/config/set` 服务、事务写入和变更通知。
  - `publish_status()`：`/config/status` 就绪状态。
- `src/cleanbot_config/src/config_registry.cpp`
  - `validate_value()`：单项配置校验。
  - `missing_required()`：缺失配置检查。
- `src/cleanbot_config/src/sqlite_config_repository.cpp`
  - `open_and_initialize()`：数据库打开和 schema 初始化。
  - `read_all()`：读取全部配置。
  - `write_values()`：校验、事务写入和 revision 更新。
  - `quick_check()`：数据库健康检查。

边界：配置节点不执行运动控制；其他节点只能通过配置接口获取配置快照。

### 4.2 下位机节点

职责：串口独占、协议编解码、ACK、状态发布、命令重试和重连刹车。

入口和关键方法：

- `src/cleanbot_hardware/src/lower_machine_node.cpp`
  - `LowerMachineNode()`：订阅 `/control/final_cmd`，创建硬件状态和命令状态发布器。
  - `onCommand()`：接收仲裁后的唯一命令。
  - `onFrame()`：处理收到的协议帧。
  - `onAckFrame()`：匹配 ACK 和命令完成事件。
  - `onRetryTimer()`：命令超时和重试。
  - `sendTrackedCommand()`：登记命令生命周期并发送。
  - `publishCommandStatus()`：发布命令执行结果。
  - `startReconnectBrakeHandshake()`：串口重连后的安全握手。
  - `onConnectionChanged()`：连接状态和安全状态转换。
- `src/cleanbot_hardware/src/lower_machine_serial.cpp`
  - `openSerial()`、`beginRead()`、`onRead()`：串口打开和接收。
  - `enqueueWrite()`、`beginWrite()`、`onWrite()`：串口发送队列。
  - `handleTransportError()`、`scheduleReconnect()`：传输错误和重连。
- `src/cleanbot_hardware/src/status_frame_parser.cpp`
  - `StatusFrameParser::parse()`：状态帧解析和字段校验。

边界：下位机节点不决定任务优先级，只执行 `/control/final_cmd`。

### 4.3 RTK 节点

职责：RTK 串口独占、NMEA 定位/航向解析、时间同步、中心点换算、NTRIP/RTCM3 和有效性发布。

入口和关键方法：

- `src/cleanbot_rtk/src/rtk_node.cpp`
  - `RtkNode()`：初始化 RTK 串口、NTRIP 和配置客户端。
  - `onRtcmBytes()`：接收 RTCM3 并写入 RTK 串口。
  - `onSerialData()`：从字节流提取 NMEA 并交给解析/同步模块。
  - `publishSample()`：发布 `/rtk/fix`，包含中心点和有效性。
  - `publishFreshness()`：定位过期心跳和失效标志。
  - `onSerialConnection()`：串口断开时清空缓存并发布失效状态。
  - `configure()`：读取 RTK/NTRIP 配置并初始化连接。
- `src/cleanbot_rtk/src/nmea_parser.cpp`：NMEA 语句解析。
- `src/cleanbot_rtk/src/rtk_sample_synchronizer.cpp`：GGA 与航向同步。
- `src/cleanbot_rtk/src/vehicle_center_transform.cpp`：天线坐标到车辆中心点换算。
- `src/cleanbot_rtk/src/rtk_validity.cpp`：固定解、时间新鲜度和坐标有效性判断。
- `src/cleanbot_rtk/src/ntrip_client.cpp`、`ntrip_protocol.cpp`、`rtcm3_frame_buffer.cpp`：差分链路。

边界：RTK 节点只发布定位事实和有效性，不直接发布车辆运动命令。

### 4.4 纠偏节点

职责：根据当前位置和目标线计算车辆控制量。

入口和关键方法：

- `src/cleanbot_control/src/tracking_node.cpp`
  - `TrackingNode()`：创建目标、RTK 订阅和控制发布器。
  - `onTarget()`：接收任务节点的目标线/目标点。
  - `onRtkFix()`：接收当前定位和航向。
  - `publishMission()`：发布任务控制命令。
  - `publishRelease()`：目标失效或任务结束时释放任务控制权。
  - `publishStatus()`：发布纠偏状态。
- `src/cleanbot_control/src/tracking_core.cpp`
  - `RtkKalmanFilter2D`：位置平滑。
  - `StraightLinePController`：横向误差、航向误差和控制量计算。

边界：纠偏节点不决定任务段顺序和命令优先级；RTK 无效时不继续使用旧坐标驱动车辆。

### 4.5 命令仲裁节点

职责：在急停、安全、人工、任务和视觉命令之间选择唯一输出，并管理命令租约。

入口和关键方法：

- `src/cleanbot_control/src/command_arbiter_node.cpp`
  - `CommandArbiterNode()`：创建各命令源订阅和最终输出。
  - `onCommand()`：接收来源命令并应用固定来源优先级。
  - `onBrush()`：独立处理滚刷命令。
  - `onMaintenanceState()`：维护安全门状态。
  - `publishIfChanged()`：租约超时检查和唯一 `/control/final_cmd` 发布。
  - `configure()`：读取命令租约配置。
- `src/cleanbot_control/src/command_arbiter_core.cpp`
  - `CommandArbiterCore::update()`：更新来源命令。
  - `CommandArbiterCore::output()`：计算当前有效输出。
  - `set_maintenance()`：维护模式安全门。
  - `set_brush()`：滚刷状态安全处理。

边界：上游不能通过消息字段自行提高优先级；下位机只信任仲裁后的最终命令。

### 4.6 任务节点

职责：任务 Action、任务状态机、控制目标发布、暂停/取消、RTK 恢复、返航和检查点。

入口和关键方法：

- `src/cleanbot_mission/src/mission_manager_node.cpp`
  - `MissionManagerNode()`：创建清扫、多路点、返航和恢复 Action。
  - `onGoal()`、`onAccepted()`：清扫任务接收和执行线程启动。
  - `onWaypointGoal()`、`onWaypointAccepted()`：多路点任务。
  - `onReturnHomeGoal()`、`onReturnHomeAccepted()`：返航。
  - `onRecoverGoal()`、`onRecoverAccepted()`：恢复任务。
  - `onSetPause()`：暂停和恢复。
  - `onRtkFix()`、`onHardwareStatus()`、`onTrackingStatus()`、`onCommandStatus()`：输入状态更新。
  - `execute()`：清扫任务主流程。
  - `executeWaypoints()`：多路点流程。
  - `executeReturnHome()`：返航流程。
  - `executeRecoverMission()`、`executeRecoveredSegments()`：恢复流程。
  - `publishTrackingTarget()`：给纠偏节点发送目标。
  - `publishMissionBrake()`、`publishSafetyBrake()`：任务刹车和安全刹车。
  - `publishBrushForSegment()`：按任务段控制滚刷。
  - `loadRecoveryCheckpoint()`、`saveCheckpoint()`、`clearCheckpoint()`：检查点选择、保存和清除。
- `src/cleanbot_mission/src/mission_checkpoint.cpp`
  - `validate_record()`：检查点字段、坐标和进度校验。
  - `MissionCheckpointStore::save()`：临时文件、同步落盘和原子替换。
  - `MissionCheckpointStore::load_latest()`：异常 JSON 和类型错误拒绝。
  - `MissionCheckpointStore::clear()`：正常完成或取消后的清理。

边界：任务节点决定任务流程和目标，不绕过仲裁节点直接控制下位机。

### 4.7 建模节点

职责：打点、区域识别、覆盖路径生成、计划存储和计划提交。

入口和关键方法：

- `src/cleanbot_modeling/src/modeling_manager_node.cpp`
  - `ModelingManagerNode()`：创建建模服务和计划 Action。
  - `onManage()`：模型/计划查询和管理。
  - `onSamplePoint()`：采样当前 RTK 点。
  - `onGeneratePlan()`：生成覆盖计划。
  - `onExecutePlan()`：校验并提交任务计划。
  - `onRtkFix()`、`onHardwareStatus()`：采样和执行前状态输入。
- `src/cleanbot_modeling/src/point_sampler.cpp`：点采样和稳定性判断。
- `src/cleanbot_modeling/src/region_recognizer.cpp`：区域识别。
- `src/cleanbot_modeling/src/coverage_planner.cpp`：覆盖路径生成。
- `src/cleanbot_modeling/src/task_plan_builder.cpp`：计划构造。
- `src/cleanbot_modeling/src/sqlite_model_repository.cpp`：模型和计划持久化。

边界：建模节点生成任务计划，不负责执行车辆运动。

### 4.8 HTTP 节点

职责：将页面请求转换成 ROS2 接口调用，并将响应转换为 HTTP/JSON。

入口和关键方法：

- `src/cleanbot_http/src/http_gateway_node.cpp`
  - `HttpGatewayNode()`：创建 HTTP 服务和 ROS2 客户端。
  - `onManualControl()`：人工控制请求。
  - `onEmergencyStop()`：急停请求。
  - `publishUserControl()`：发布人工控制意图。
  - `startServer()`、`stopServer()`：HTTP 生命周期。
  - `publishControl()`：控制结果和状态返回。
- `src/cleanbot_http/src/http_control_router.cpp`：HTTP 路由和请求体校验。
- `src/cleanbot_http/src/http_session.cpp`：HTTP 会话读写。
- `src/cleanbot_http/src/service_response_mapping.cpp`：ROS2 响应到 HTTP 结果。
- `src/cleanbot_http/src/vehicle_state_json.cpp`：车辆状态 JSON 序列化。

边界：HTTP 节点不直接访问串口，不直接改变任务内部状态，只调用 ROS2 Service/Action 或发布控制意图。

## 5. 通信接口讲解方式

### Topic

用于连续状态和实时数据：RTK、硬件状态、纠偏状态、最终车辆命令、配置状态。

### Service

用于短请求短响应：配置读写、暂停、急停、人工控制、模型管理。

### Action

用于有持续时间、反馈和取消的任务：清扫、多路点、返航和恢复。

评审时每个接口都标出：发布方、订阅方、请求方、响应方、反馈来源、取消路径和超时行为。

## 6. 一条清扫任务的源码追踪

```text
页面请求
  → HTTP 路由和参数校验
  → 建模节点 onExecutePlan()
  → 任务节点 onGoal() / onAccepted()
  → 任务节点 execute()
  → 任务节点 publishTrackingTarget()
  → 纠偏节点 onTarget() / onRtkFix()
  → 纠偏节点 publishMission()
  → 仲裁节点 onCommand() / publishIfChanged()
  → 下位机节点 onCommand()
  → 串口发送、ACK、状态发布
  → 任务节点 onCommandStatus()
  → Action feedback/result
  → HTTP 响应
```

每个箭头都对应一个接口和一个源码方法，评审中从请求进入开始逐段跳转源码，不按目录孤立讲解。

## 7. 异常、安全和恢复

- 配置未就绪：任务入口拒绝执行，仲裁节点输出安全刹车。
- RTK 无效或过期：RTK 节点发布失效状态，任务节点等待，纠偏节点释放运动控制；恢复后重新定位并重新规划当前段。
- 下位机断线：串口节点清空发送状态，重连后先执行刹车握手，再恢复可接受命令。
- 命令租约超时：仲裁节点停止使用过期命令，输出安全结果。
- 急停/安全门：覆盖人工和任务运动命令；滚刷保持关闭，解除后不会自动恢复滚刷。
- 暂停/取消：任务节点停止目标和滚刷输出，等待命令状态确认后结束或恢复。
- 检查点损坏：拒绝加载，不让异常 JSON 进入任务状态机；保存采用临时文件和原子替换。
- 任务恢复：优先选择匹配的返航检查点或清扫检查点，使用最新 RTK 重新定位后执行未完成任务段。

## 8. 方法注释标准

关键方法统一说明：目的、前置条件、输入、输出、副作用、异常/超时和并发约束。优先整理以下类型：Action 接收和执行入口、状态回调、控制输出、串口边界、检查点读写和 HTTP 请求转换。

## 9. 评审验收标准

评审完成时应能回答：

1. 任意一个节点为什么存在，它不负责什么。
2. 任意一条控制命令从哪里来，经过哪些安全边界，最终如何到达下位机。
3. 一个任务如何接受、执行、反馈、暂停、取消和结束。
4. RTK、串口、配置和检查点异常时，车辆为什么不会继续使用危险的旧状态。
5. 每个结论能否回到具体源码文件和方法。

