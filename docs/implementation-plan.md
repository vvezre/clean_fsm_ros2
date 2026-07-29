# clean_fsm C++ ROS2 Humble Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Every behavior change follows test-first development.

**Goal:** Replace the Python/Redis clean_fsm runtime with a C++17 ROS2 Humble system while preserving lower-machine protocol, RTK/NTRIP behavior, cleaning workflows, HTTP contracts, and MQTT contracts.

**Architecture:** The migration uses one ROS2 package per responsibility. Hardware ports have one owner, mission execution uses Actions, transient state uses Topics and node memory, commands use Services or Actions, and persistent records use SQLite. The old project remains read-only migration reference.

**Tech Stack:** C++17, ROS2 Humble, ament_cmake, rclcpp, Boost.Asio, OpenCV, SQLite3, libcurl, Paho MQTT C++, nlohmann/json, GTest.

---

## Stage 1: Workspace and contracts

- Create the Humble `colcon` workspace.
- Define ROS messages, services, and actions before node implementation.
- Create `cleanbot_common` for dependency-free domain types and math.
- Add repository contract tests and Humble build scripts.
- Freeze Python source locations and compatibility boundaries.

**Exit criteria:** Contract tests pass locally; package metadata is valid; `colcon build` instructions are complete for Ubuntu 22.04.

## Stage 2: Lower-machine gateway

- Port mixed status/ACK buffering, frame extraction, status parsing, 21-byte command encoding, A1 retry, completion deduplication, and A2 confirmation.
- Use one Boost.Asio serial owner and separate read/write queues.
- Publish `HardwareStatus`; subscribe only to the final arbitrated command.
- Add byte-for-byte golden tests from captured Python protocol frames.

**Exit criteria:** Recorded input frames produce equivalent status values; command/A1 and completion/A2 loss scenarios recover without duplicate vehicle actions or duplicate mission advancement.

## Stage 3: RTK and NTRIP

- Port NMEA buffering, GGA/HPR parsing, timestamps, and fix quality.
- Integrate NTRIP connection and RTCM forwarding inside the RTK serial owner.
- Publish `RtkFix` and connection diagnostics.
- Add recorded NMEA and reconnect tests.

**Exit criteria:** Position, heading, fix quality, and NTRIP recovery match recorded Python behavior.

**Code status (2026-07-14):** Implemented locally. The package uses one `RtkSerial` owner, an internal callback-based `NtripClient`, bounded NMEA buffering, GGA/HPR/THS validation, fresh sample synchronization, and the legacy-compatible 0.10 m forward plus 0.18 m right vehicle-center transform. Contract and dependency-free runtime tests pass. Dashing compilation, serial replay, network-loss testing, and hardware acceptance remain pending.

## Stage 4: Tracking and control arbitration

- Port local coordinate conversion, 2D Kalman filter, CTE calculation, and straight-line P control.
- Introduce command arbitration with priority: emergency stop, safety stop, mission, manual, vision.
- Keep the lower-machine gateway as the only serial writer.
- Compare C++ outputs against Python golden datasets with numeric tolerances.

**Exit criteria:** Tracking outputs are equivalent and all competing command sources resolve deterministically.

## Stage 5: Mission and safety

- Port lifecycle state transitions to explicit C++ events and enums.
- Implement cleaning, waypoint navigation, docking, return-home, stop, and resume as Actions.
- Port RTK watchdog, edge guard, low-battery return, and hardware timeout handling.
- Replace thread stop globals with Action cancellation and atomic safety state.

**Exit criteria:** Every normal and abnormal workflow reaches the expected terminal state and always brakes on cancellation or fault.

## Stage 6: Modeling and persistence

- Port point sampling, layout planning, task generation, and model execution.
- Store configuration, models, task records, alarms, and statistics in SQLite.
- Do not recreate Redis semantics or a global key-value store.
- Add migration tooling for existing JSON task files.

**Exit criteria:** Existing model fixtures generate equivalent task segments and persist across restart.

## Stage 7: HTTP and MQTT compatibility

- Implement the existing `/vehicle/...` contracts in the C++ gateway.
- Translate HTTP and MQTT requests into ROS Services and Actions.
- Build response payloads from subscribed state snapshots.
- Preserve error codes and fields used by the mini-program and cloud platform.

**Exit criteria:** Contract tests pass against captured requests and responses without requiring frontend changes.

## Stage 8: Vision and integration

- Port OpenCV bright-band and line guidance algorithms.
- Add ROS image/control interfaces without allowing direct serial writes.
- Add launch, systemd, configuration, diagnostics, and log rotation.
- Run protocol replay, hardware-in-loop, field scenarios, and 72-hour soak testing.

**Exit criteria:** Full robot workflows pass and no process shows unbounded memory growth or unsafe restart behavior.

## Stage 9: Debug diagnostics and full-function test platform

- Collect `/rosout`, node health, hardware, RTK, mission, control, storage, and gateway diagnostics.
- Route logs into one searchable and rotating log stream per ROS2 node.
- Aggregate repeated faults into active/resolved issues with severity, first occurrence, last occurrence, and count.
- Provide a browser-based debug dashboard for node state, logs, active faults, and historical faults.
- Provide guarded test buttons for communication, RTK, motion, brush, navigation, cleaning tasks, pause/resume/cancel, and emergency stop.
- Execute tests through typed ROS2 Services and Actions without bypassing command arbitration or safety handling.
- Record every test's prerequisites, steps, expected result, actual result, timeout, cleanup action, and final verdict.

**Exit criteria:** An engineer can locate a node fault, inspect its related logs, run a repeatable subsystem or end-to-end test, and export a complete diagnostic report without using terminal commands.

## Delivery order

1. `cleanbot_interfaces`, `cleanbot_common`, `cleanbot_bringup`
2. `cleanbot_hardware`
3. `cleanbot_rtk`
4. `cleanbot_control`, `cleanbot_safety`
5. `cleanbot_mission`
6. `cleanbot_modeling`, `cleanbot_storage`
7. `cleanbot_gateway`, `cleanbot_cloud`
8. `cleanbot_vision`, integration and field validation
9. `cleanbot_diagnostics`, `cleanbot_test_runner`, `cleanbot_debug_gateway`

## Global rules

- Application code is C++17; ROS launch files use the Humble launch runtime.
- No ROS API is called from dependency-free algorithm libraries.
- No node except the relevant hardware owner opens a serial device.
- No command source writes directly to the lower-machine gateway.
- Every ported algorithm receives golden input/output tests before implementation.
- A stage is complete only after its tests and documented acceptance checks pass.
