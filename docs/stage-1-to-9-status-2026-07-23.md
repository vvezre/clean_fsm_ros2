# clean_fsm ROS2 S1-S9 状态审查

审查日期：2026-07-23

## 判定口径

- **代码完成**：接口、核心实现和自动化测试已经存在。
- **环境完成**：已在树莓派 ROS2 Humble 下编译。
- **功能完成**：已使用真实串口、RTK、下位机或车辆完成业务验收。

不能用“代码存在”代替“功能已经真机完成”。

## 当前验证证据

- Windows 本地：179 项 Python/依赖无关 C++ 回归测试通过。
- 树莓派：现有 9 个功能包全量编译成功，全部 `rc=0`。
- 树莓派已有测试记录：78 项，0 错误、0 失败。
- 树莓派目前没有 `cleanbot_modeling` 和 `cleanbot_gateway`。
- 尚未执行完整启动、真实下位机、真实 RTK/NTRIP 和真车业务验收。

## S1-S9 工程目录与关键代码

工程源码根目录：`src/`

### 功能包职责总览

| 功能包 | 主要作用 | 在业务链路中的位置 |
|---|---|---|
| `cleanbot_bringup` | 保存整机启动文件和启动参数，一次启动下位机、RTK、控制、任务、建模和 HTTP 等节点 | 系统总启动入口 |
| `cleanbot_interfaces` | 定义节点之间统一使用的消息、Service 和 Action 数据格式，本身不执行业务 | 所有功能包共同依赖的数据契约 |
| `cleanbot_common` | 保存多个功能包共同使用的基础能力，目前主要是统一 QoS 通信参数 | ROS2 通信公共基础 |
| `cleanbot_config` | 集中管理系统配置，校验配置项，并使用 SQLite 持久化；业务节点通过配置接口读取参数，不直接访问数据库 | 系统初始化和参数管理 |
| `cleanbot_hardware` | 独占下位机串口，把 ROS2 车辆命令编码成串口帧发送，并把下位机状态帧解析为 ROS2 硬件状态 | ROS2 与下位机之间的硬件网关 |
| `cleanbot_rtk` | 独占 RTK 串口，解析定位和航向，连接 NTRIP 获取差分数据，判断固定解，并将天线坐标转换为车体中心坐标 | 为纠偏、任务和建模提供定位数据 |
| `cleanbot_control` | 根据 RTK 和目标路径计算横向偏差、航向误差及转向量；同时对任务、摇杆、视觉和急停命令进行统一选择 | 车辆运动控制核心 |
| `cleanbot_mission` | 执行自动清扫和多路点任务，管理任务段、转向、直行、暂停、恢复、取消、RTK 恢复和触边处理 | 上层任务调度和安全流程 |
| `cleanbot_modeling` | 读取 RTK 进行现场打点，识别区域和连接关系，生成往复式清扫路径，并用 SQLite 保存模型和规划结果 | 清扫模型创建和路径生成 |
| `cleanbot_http` | 提供局域网 HTTP 控制入口，把网页或小程序的请求转换为 ROS2 控制消息 | 用户侧局域网接入层 |
| `cleanbot_gateway` | 接收 MQTT 云端消息并转换为 ROS2 命令，同时负责云端消息编解码 | 云平台远程接入层 |
| `cleanbot_vision` | 计划用于相机采集、视觉识别和视觉控制；当前尚未创建 | 后续视觉能力 |
| S9 调试平台相关包 | 计划汇总节点日志、健康状态和异常，并提供整机功能测试按钮及测试报告 | 后续诊断和测试工具 |

主要业务数据流：

```text
HTTP/云平台请求
    -> cleanbot_http / cleanbot_gateway
    -> cleanbot_mission（组织清扫或多路点任务）
    -> cleanbot_control（计算纠偏并选择最终车辆命令）
    -> cleanbot_hardware（编码串口命令）
    -> 下位机执行

RTK设备
    -> cleanbot_rtk（解析、差分、有效性判断、车体中心换算）
    -> cleanbot_control（实时纠偏）
    -> cleanbot_mission（任务位置和安全判断）
    -> cleanbot_modeling（现场打点和路径建模）

下位机状态
    -> cleanbot_hardware（解析状态帧）
    -> cleanbot_mission / cleanbot_control（完成、触边和运动状态判断）
```

### S1：ROS2 工程骨架与配置

- 工程启动包：`src/cleanbot_bringup`
  - 总启动入口：`launch/cleanbot.launch.py`
  - 系统启动参数：`config/system.yaml`
- ROS2 接口包：`src/cleanbot_interfaces`
  - 消息定义：`msg/*.msg`
  - 服务定义：`srv/*.srv`
  - 长任务定义：`action/*.action`
- 公共能力包：`src/cleanbot_common`
  - QoS 统一定义：`include/cleanbot_common/qos_profiles.hpp`
- 配置管理包：`src/cleanbot_config`
  - 配置节点入口：`src/config_manager_node.cpp`
  - 配置项注册：`src/config_registry.cpp`
  - SQLite 存储：`src/sqlite_config_repository.cpp`
  - 业务节点配置客户端：`src/config_client.cpp`

### S2：下位机串口网关

- 功能包目录：`src/cleanbot_hardware`
- 节点入口：`src/lower_machine_node.cpp`
- 串口连接与收发：`src/lower_machine_serial.cpp`
- 下发命令编码：`src/command_encoder.cpp`
- 23 字节状态帧解析：`src/status_frame_parser.cpp`
- 分包、粘包和噪声重同步：`src/frame_buffer.cpp`
- ACK 编解码：`src/ack_codec.cpp`
- 命令 ACK、完成和超时生命周期：`src/command_lifecycle.cpp`
- 命令优先级及写队列：`src/write_queue.cpp`
- 串口就绪与重连状态：`src/gateway_readiness.cpp`
- 自动化测试：`test/test_*.cpp`

### S3：RTK 与 NTRIP

- 功能包目录：`src/cleanbot_rtk`
- 节点入口与整体链路：`src/rtk_node.cpp`
- RTK 串口所有者：`src/rtk_serial.cpp`
- NMEA 行缓存：`src/nmea_line_buffer.cpp`
- GGA、HPR、THS 解析：`src/nmea_parser.cpp`
- 多类 RTK 数据同步：`src/rtk_sample_synchronizer.cpp`
- RTK 有效性和固定解判断：`src/rtk_validity.cpp`
- 天线坐标转换为车体中心：`src/vehicle_center_transform.cpp`
- NTRIP 客户端：`src/ntrip_client.cpp`
- NTRIP 协议处理：`src/ntrip_protocol.cpp`
- RTCM3 差分帧缓存：`src/rtcm3_frame_buffer.cpp`
- 自动化测试：`test/test_*.cpp`

### S4：纠偏与统一命令

- 功能包目录：`src/cleanbot_control`
- RTK 直线纠偏节点：`src/tracking_node.cpp`
- Kalman 滤波、航向误差、CTE 和 P 控制：`src/tracking_core.cpp`
- 统一命令节点：`src/command_arbiter_node.cpp`
- 命令优先级、超时和急停核心：`src/command_arbiter_core.cpp`
- 摇杆数值转换：`src/joystick_mapper.cpp`
- 自动化测试：`test/test_tracking_core.cpp`、`test/test_command_arbiter_core.cpp`、`test/test_joystick_mapper.cpp`

### S5：任务与安全

- 功能包目录：`src/cleanbot_mission`
- 任务节点入口：`src/mission_manager_node.cpp`
- 自动清扫任务状态机：`src/mission_state_machine.cpp`
- 多路点路径计划：`src/waypoint_plan.cpp`
- 直行触边分类与处理：`src/straight_edge_guard.cpp`
- 自动清扫 Action：`src/cleanbot_interfaces/action/ExecuteCleaning.action`
- 多路点 Action：`src/cleanbot_interfaces/action/NavigateWaypoints.action`
- 暂未实现服务端的 Action：`src/cleanbot_interfaces/action/Dock.action`、`ReturnHome.action`
- 自动化测试：`src/cleanbot_mission/test/test_*.cpp`

### S6：打点建模与持久化

- 功能包目录：`src/cleanbot_modeling`
- 建模节点入口：`src/modeling_manager_node.cpp`
- 静止状态 RTK 均值采样：`src/point_sampler.cpp`
- 子区域和点类型识别：`src/region_recognizer.cpp`
- 几何计算及多边形自相交检查：`src/geometry.cpp`
- 往复式覆盖路径规划：`src/coverage_planner.cpp`
- 模型转换为任务段：`src/task_plan_builder.cpp`
- SQLite 模型、版本和规划结果存储：`src/sqlite_model_repository.cpp`
- 内部模型结构：`include/cleanbot_modeling/model_types.hpp`
- 对外接口：`src/cleanbot_interfaces/srv/*Model*.srv`、`GenerateCleaningPlan.srv`、`SampleModelPoint.srv`
- 自动化测试：`src/cleanbot_modeling/test/test_*.cpp`

### S7：HTTP 与 MQTT 兼容

- HTTP 功能包目录：`src/cleanbot_http`
  - HTTP 节点入口：`src/http_gateway_node.cpp`
  - 监听端口和会话接入：`src/http_server.cpp`
  - 单个 HTTP 会话：`src/http_session.cpp`
  - URL 路由到 ROS2 控制命令：`src/http_control_router.cpp`
  - 摇杆请求顺序保护：`src/joystick_sequence_guard.cpp`
- MQTT 功能包目录：`src/cleanbot_gateway`
  - MQTT 节点入口：`src/cloud_gateway_node.cpp`
  - 云端控制命令转换：`src/cloud_command.cpp`
  - MQTT 消息编解码：`src/cloud_message_codec.cpp`
- 自动化测试：`src/cleanbot_http/test/test_*.cpp`、`src/cleanbot_gateway/test/test_*.cpp`

### S8：视觉与整机集成

- 当前没有对应源码功能包，也没有 `cleanbot_vision` 目录。
- 后续建议新建：`src/cleanbot_vision`，放置相机接入、图像处理和视觉控制节点。
- 整机部署入口仍由：`src/cleanbot_bringup/launch/cleanbot.launch.py` 统一启动。

### S9：调试诊断与功能测试平台

- 当前只有设计文档，尚无对应源码功能包。
- 设计文档：`docs/superpowers/specs/2026-07-23-stage-9-debug-diagnostics-platform-design.md`
- 后续建议拆分为诊断节点、测试执行节点和 Debug 页面，统一接收各节点日志、健康状态和异常事件。

## S1：ROS2 工程骨架与配置

**状态：主体完成，部署验收未完成。**

已完成：

- `cleanbot_interfaces`、`cleanbot_common`、`cleanbot_config`、`cleanbot_bringup`；
- ROS2 消息、Service、Action 和统一 QoS；
- SQLite 系统配置、配置就绪状态和配置变更；
- 默认 launch 已连接下位机、RTK、控制、任务、建模和 HTTP 节点；
- S1 相关包已在树莓派编译。

未完成：

- 使用真实配置完成一次完整 `ros2 launch` 启动验收；
- 验证缺少必填配置、数据库损坏、磁盘不足和节点重启场景；
- 清理仓库中的 `.obj`、`__pycache__` 等生成文件；
- 更新已过期的 README 阶段说明。

## S2：下位机串口网关

**状态：代码和协议测试完成，硬件验收未完成。**

已完成：

- 23 字节固定状态帧解析；
- 21 字节命令帧和 sequence；
- A1 ACK、A2 上位机确认、完成状态；
- 分包、粘包、噪声重新同步；
- 优先级写队列、连续命令合并、重试和断线刹车握手；
- 树莓派编译和协议自动化测试。

未完成：

- 下位机固件按新 ACK/完成协议联调；
- 真实串口分包、粘包、断线、重连和掉电测试；
- 丢 ACK、重复 ACK、延迟完成和串口拥堵测试；
- 真实状态字段范围、符号和单位核对；
- 真车运动与刹车验证；
- 状态帧没有 CRC，当前只能检查帧头、帧尾和固定长度，无法可靠识别合法长度的内容位翻转。

## S3：RTK 与 NTRIP

**状态：代码和算法测试完成，现场定位验收未完成。**

已完成：

- 单 RTK 串口所有者；
- GGA、HPR/THS 同步；
- 车辆中心点偏移计算；
- 固定解、新鲜度和有效状态；
- 异步 NTRIP、RTCM3 完整帧转发和重连基础；
- 树莓派编译和协议自动化测试。

未完成：

- 真实 RTK 接收机串口读取和输出格式确认；
- 真实 NTRIP 账号、鉴权、RTCM 回注和断网恢复测试；
- 每台车辆的天线到车体中心偏移标定；
- 航向方向、安装方向和中心点黄金数据验证；
- RTK 串口回放、长时间丢星和固定解反复切换测试；
- 清理 `rtk_node.cpp` 中残留的中文乱码注释。

## S4：纠偏与统一命令

**状态：核心代码完成，车辆参数整定未完成。**

已完成：

- 二维 Kalman RTK 滤波；
- 直线 P 控制、航向误差、CTE 和终点判断；
- 摇杆映射；
- emergency、safety、manual、mission、vision 命令优先级；
- 命令租约、软件急停和滚刷独立控制；
- 树莓派编译和核心自动化测试。

未完成：

- 真车确认转向正负方向；
- 整定 Kalman 参数、航向增益、CTE 增益、输出限幅和终点容差；
- 长直线、低速、临近终点和 RTK 抖动场景测试；
- `/control/vision_cmd` 只有接收入口，没有视觉控制节点；
- `ManualControl.srv`、`EmergencyStop.srv` 目前只有接口定义，没有对应 ROS2 Service 服务端。

## S5：任务与安全

**状态：自动清扫和多路点核心完成，完整任务族未完成。**

已完成：

- `ExecuteCleaning` Action；
- `NavigateWaypoints` Action；
- 单点、多点、有限循环和连续循环；
- 转向完成关联、直行 generation 关联；
- 暂停、继续、取消和 RTK 恢复；
- 正常到达、越过终点和异常触边的核心判断；
- 异常触边后等待人工确认继续；
- 树莓派编译和状态机自动化测试。

未完成：

- `Dock.action` 只有接口，没有回充执行节点；
- `ReturnHome.action` 只有接口，没有返航执行节点；
- 低电量返航；
- 任务断电恢复和运行检查点持久化；
- 统一报警记录与用户确认入口；
- 触边传感器抖动阈值和真车边界测试；
- 自动清扫、多路点、暂停恢复、RTK 丢失的整车联调。

## S6：打点建模与持久化

**状态：本地第一版代码存在，但尚不能判定完成。**

已完成：

- RTK 均值采样；
- 模型、区域组、点、子区域和连接段数据结构；
- 基础自动识别；
- 往复式覆盖路径和重叠量计算；
- 模型版本、计划和 SQLite 存储；
- 模型级统一坐标原点，所有区域组共用同一局部坐标系；
- SQLite schema v2及旧v1模型点坐标自动迁移；
- 生成计划并提交 `ExecuteCleaning`；
- 本地自动化测试。

未完成：

- `cleanbot_modeling` 尚未同步到树莓派编译；
- 子区域和区域组之间按已确认连接段生成通行路径；
- 区域组连接段及连接图寻路；
- 旧模型坐标迁移和旧计划失效处理；
- 四条主边拟合；
- 容差级重复点检查；
- 点到主边距离判断和弧边辅助点归类；
- 多连接段、不规则区域的稳定分割；
- Action 服务端拒绝任务时向调用方返回真实拒绝结果；
- 删除请求速度覆盖，统一使用全局自动速度；
- 建模、识别、预览、确认和执行的用户页面。

说明：自相交检查已经存在于 `recognize_group()` 和 `is_simple_polygon()`，因此不再列为完全缺失；当前不足是复杂现场识别的稳定性。

## S7：HTTP 与 MQTT 兼容

**状态：HTTP 手动控制完成，业务接口和云端兼容未完成。**

已完成：

- `GET /vehicle/joystickMove/...`；
- `GET /vehicle/parking`；
- HTTP 异步连接、请求超时、退出和端口冲突处理；
- 摇杆 `sessionId + sequence` 防止旧命令乱序覆盖；
- HTTP 包已在树莓派编译和测试；
- MQTT 摇杆、停车的本地核心代码。

未完成：

- `cleanbot_gateway` 尚未同步到树莓派编译；
- MQTT 节点尚未加入默认 launch；
- MQTT 真实 Broker 鉴权、断线重连和云端联调；
- 云端/小程序需要的设备状态查询；
- 自动清扫、多路点、暂停、继续、取消接口；
- 打点、模型、识别、预览和执行接口；
- 命令、任务和异常结果查询；
- 旧 Python 可用接口与新 C++ 接口的完整兼容测试。

## S8：视觉与整机集成

**状态：未开始。**

未完成：

- `cleanbot_vision` 功能包；
- 摄像头驱动和图像 Topic；
- 旧 OpenCV 亮带、边线或视觉导航算法迁移；
- 视觉控制命令发布；
- systemd 自启动和故障拉起；
- 正式日志轮转；
- 协议回放、硬件在环、整车场景测试；
- 72 小时稳定性、内存和重启测试；
- 正式部署包和版本升级流程。

## S9：调试诊断与整机功能测试平台

**状态：设计完成，代码未开始。**

未完成：

- `cleanbot_diagnostics`；
- 每节点日志收集、查询和轮转；
- 节点心跳与整机健康摘要；
- 异常去重、汇总、恢复和 `diagnostics.db`；
- `cleanbot_test_runner`；
- 无运动测试、动作测试和完整业务测试；
- 测试超时、自动刹车和报告导出；
- `cleanbot_debug_gateway` 和浏览器调试页面。

## 建议实施顺序

1. 继续完成 S6 连接段路径和区域识别增强。
2. 将 S6、S7 MQTT 同步到树莓派，完成全工程编译。
3. 完成 S2-S5 的下位机、RTK 和真车闭环联调。
4. 补齐 S7 小程序和云平台真正需要的业务接口。
5. 实施 S8 视觉和部署集成。
6. S9 保持为正式阶段，但可提前实施 S9.1 日志与节点健康，帮助后续真车联调定位问题。

## 总结

当前不是“S1-S7 全部完成”，更准确的表述是：

- S1-S4：主体代码完成，硬件和现场验收未完成；
- S5：清扫与多路点核心完成，返航、回充、低电量和断点恢复未完成；
- S6：第一版代码完成，但坐标与跨区域路径需要修正；
- S7：仅手动控制 HTTP 链路完成；
- S8：未开始；
- S9：只有设计。
