# clean_fsm Migration Baseline

## Source system

- Read-only reference: the legacy Python project (not included in this repository)
- Entry point: `main.py:9041 main()`
- Target runtime: Raspberry Pi 4B, Ubuntu 22.04, ROS2 Humble, C++17
- External contracts to preserve: lower-machine serial protocol, RTK/NMEA, NTRIP, `/vehicle/...` HTTP API, MQTT topics and payloads

## Primary migration anchors

| Responsibility | Python reference |
| --- | --- |
| Lower-machine status reader | `main.py:8425 listenerSlavePort()` |
| Lower-machine frame parser | `main.py:1590 _apply_lower_machine_status_frame()` |
| RTK listener | `main.py:8909 listenerRTK()` |
| RTK manager | `RTKDataManager.py:57 RTKDataManager` |
| RTK watchdog | `main.py:8859 rtk_watchdog_thread()` |
| Tracking callback | `main.py:5859 observer_go_correct()` |
| Point-to-point execution | `main.py:5189 pointToPointByRTK()` |
| Automatic cleaning | `main.py:4102 autoDriveByRTKThread()` |
| Multi-waypoint execution | `main.py:5351 multiGoToPointThread()` |
| Turn execution | `main.py:7384 turn()` |
| Kalman filter | `rtk_path_tracking.py:121 RTKKalmanFilter2D` |
| Straight-line controller | `rtk_path_tracking.py:231 StraightLinePController` |
| Layout task generation | `layout_planner.py:693 create_task_by_panel_layout()` |
| Runtime state machine | `robot_fsm.py:91 RobotLifecycleFSM` |

## State replacement rules

| Existing Redis/global usage | C++ ROS2 owner |
| --- | --- |
| Live RTK and hardware state | Publishing node memory and Topics |
| Mission progress and cancellation | Mission Action state and feedback |
| Control commands | Services, Actions, and command arbitration Topics |
| Runtime lifecycle | `mission_manager_node` state machine |
| Models, configuration, records | SQLite repository components |
| HTTP/MQTT status payloads | `state_aggregator_node` snapshot |

## Stage 1 boundaries

Stage 1 creates interfaces and dependency-free foundations only. It does not open hardware ports, control a vehicle, claim numerical equivalence, or alter the Python project.

## Stage 3 RTK contract

| Concern | New owner or rule |
| --- | --- |
| RTK device | `cleanbot_rtk::RtkSerial` is the only serial owner |
| NTRIP | `NtripClient` owns only TCP; RTCM is returned through `RtkSerial::enqueueWrite()` |
| Raw position | `RtkFix.raw_lat/raw_lon` contains the antenna GGA coordinate |
| Vehicle position | `RtkFix.lat/lon` contains the calculated vehicle-center coordinate |
| Center offset | Preserve the Python order: 0.10 m along heading, then 0.18 m to the right |
| Heading | GGA and HPR/THS are paired by GNSS UTC and receive age; stale heading is not reused indefinitely |
| Invalid heading | Retain the last valid center coordinate and publish `center_valid=false` |
| Safety/control | RTK publishes facts only; watchdog, braking, tracking, and command arbitration belong to later packages |

Stage 3 intentionally removes the old unlimited last-heading reuse, mixed RTK serial ownership, and NTRIP-to-serial coupling. Target-environment build and hardware validation are not yet claimed.
