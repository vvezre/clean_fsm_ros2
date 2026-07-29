# clean_fsm ROS2

C++17 and ROS2 Humble rewrite of the clean_fsm robot-side service.

## Target

- Raspberry Pi 4B or another ARM64/x86_64 computer
- Ubuntu 22.04
- ROS2 Humble
- C++17 application code

The legacy Python project (not included in this repository) is a read-only migration reference. New runtime state is exchanged through ROS interfaces. SQLite is the only runtime source for persistent system configuration.

## Packages

- `cleanbot_interfaces`: shared messages, services, and actions
- `cleanbot_common`: shared ROS2 QoS profiles and common communication contracts
- `cleanbot_config`: system configuration validation, SQLite persistence, readiness state, and configuration services
- `cleanbot_bringup`: launch and system configuration
- `cleanbot_hardware`: lower-machine serial gateway and protocol implementation
- `cleanbot_rtk`: RTK serial ownership, NMEA synchronization, vehicle-center conversion, and NTRIP
- `cleanbot_control`: RTK straight-line tracking, joystick mapping, and command arbitration
- `cleanbot_mission`: cleaning-segment sequencing, turn completion matching, pause, cancellation, and RTK recovery
- `cleanbot_modeling`: RTK point sampling, sub-area recognition, even-lane coverage planning, SQLite model versions, and executable cleaning-plan generation
- `cleanbot_http`: C++ HTTP gateway for user control, vehicle state, mission control, and modeling-plan execution

Additional packages are introduced stage by stage according to `docs/implementation-plan.md`.

## Configuration

- `cleanbot_config` is the only package that reads or writes the configuration database.
- Business nodes obtain typed values through `ConfigClient`; they do not access SQLite directly.
- `src/cleanbot_bringup/config/system.yaml` contains bootstrap values only: database path, profile, and minimum free space.
- Required hardware and RTK values have no silent defaults. The system publishes `config_ready=false` until they are configured and valid.
- `motion.base_forward_speed` is the shared automatic-driving speed. Its default is 350, its valid range is 50 to 600, and valid updates take effect immediately.
- Joystick speed remains independent through `motion.manual_max_speed` and valid changes take effect immediately.

## HTTP control and business entry

The C++ gateway preserves the old control URLs:

```text
GET /vehicle/joystickMove/{distance}/{dirX}/{dirY}
GET /vehicle/parking
```

It also exposes the first versioned business routes:

```text
GET  /api/v1/vehicle/state
POST /api/v1/mission/pause
POST /api/v1/mission/resume
POST /api/v1/modeling/execute-plan?planId={id}&brushSpeed={positiveInteger}
```

The server listens on port `7899` by default. It uses the existing `JoystickMapper` and
publishes `/control/manual_cmd` or `/control/emergency_cmd`; it never accesses the serial
port directly. Successful requests retain the old plain-text response `1`. Invalid input
is rejected before any ROS2 command is published. If joystick requests stop, the existing
500 ms manual command lease makes the command arbiter output a brake.

`MissionManagerNode` publishes the authoritative, transient-local `/vehicle/state`
snapshot at 5 Hz. The business routes adapt HTTP requests to `/vehicle/state`,
`/mission/set_pause`, and `/modeling/execute_plan`; service failure codes are preserved
in JSON responses. Execute-plan returns HTTP `202` only after the cleaning Action server
has accepted the generated goal. The gateway also implements the ROS2
`/control/manual` and `/control/emergency_stop` service servers.

The current first-batch gateway uses one Asio worker, so business requests that
wait for ROS services are serialized. The 3000 ms Beast timeout applies to
socket read/write operations, not total queued request latency; runtime
concurrency measurements are required before changing this bridge to a fully
asynchronous response path.

Before building on Ubuntu 22.04, ensure the Boost system development package is present:

```bash
sudo apt update
sudo apt install libboost-system-dev
```

`http.enabled`, `http.listen_address`, and `http.port` are stored through the central
configuration service. The default launch starts `http_gateway_node`; cloud MQTT is not
part of the current default runtime path.

## Current migration status

Stage 1 is complete. Stage 2 includes classified ACK retry, priority/coalescing serial writes, and a reconnect brake handshake that requires both a brake ACK and a valid status frame. Stage 3 includes one RTK serial owner, monotonic freshness, GGA/HPR/THS synchronization, vehicle-center conversion, asynchronous NTRIP, optional receiver persistence, and complete CRC-checked RTCM3 forwarding. Stage 4 includes the dependency-free Kalman/P tracking core, joystick mapping, `tracking_node`, and `command_arbiter_node`.

Stage 5 includes `ExecuteCleaning` and `NavigateWaypoints` Action servers. Waypoint navigation supports a single point, ordered multi-point navigation, finite closed loops, and continuous loops. Every leg starts from the latest valid vehicle-center RTK pose, then reuses the existing finite-turn, straight-line tracking, edge confirmation, pause/cancel, and RTK recovery chain. Normal waypoint start, transition, completion, and cancellation do not modify the independently controlled brush. Automatic movement continues to use `motion.base_forward_speed`; waypoint goals do not contain a speed field.

The modeling flow now stores the user-selected `boundary` or `connection` type for every sampled point. Ordered boundary runs are closed into arbitrary simple sub-areas, boundary corners and assist points are classified, and manually paired connection points create a directed sub-area connector. A unique high-confidence unordered boundary candidate is auto-confirmed; ambiguous candidates remain `needs_confirmation`. The modeling SQLite schema is v3, with v1/v2 migration, recognition status, connector ownership, and round-trip persistence. Multi-area plans must use confirmed connectors instead of drawing a direct line across the blank area.

The mission executor matches finite turns by producer `request_id` and straight-line completion by tracking `generation`, controls cleaning-task brush output separately, and restarts the current segment after pause or RTK recovery. Vehicle commands carry business correlation, source activation, operator intent, and manual steering. The hardware gateway publishes `/hardware/command_status` for queued, sent, acknowledged, rejected, timed-out, transport-lost, superseded, and completed commands.

Stage 5 also includes `ReturnHome` plus separate cleaning and return-home JSON
checkpoints. The 2026-07-28 local batch adds the authoritative vehicle snapshot,
live user-control services, the first versioned business HTTP routes, and actual
mission Action goal-acceptance reporting for saved-plan execution. Native ROS2
build/test and stationary runtime smoke verification for this local batch remain
pending.

## RTK output contract

- `raw_lat` and `raw_lon`: antenna GGA coordinate.
- `lat` and `lon`: vehicle-center coordinate after 0.10 m forward and 0.18 m right offset.
- `center_valid`: true only when both the position and a fresh synchronized heading are valid.
- `serial_connected`: current RTK serial transport state.
- `coordinate_valid`: true only for a parsed coordinate with nonzero GGA quality.
- `fixed_valid`: true only for connected, fresh, fixed-quality vehicle-center data.
- If heading expires, the last center coordinate is retained but `center_valid` is false.
- Fix quality and NTRIP status are published as measured; the RTK node does not brake or issue motion commands.

## Build on Ubuntu 22.04

```bash
cd /workspace/clean_fsm_ros2
./scripts/build_humble.sh
source install/setup.bash
```

## Repository checks

```bash
python3 -m unittest discover -s tests -v
```
