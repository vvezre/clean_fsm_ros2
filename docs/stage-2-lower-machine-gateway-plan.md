# Stage 2 Lower-Machine Gateway Implementation Plan

**Goal:** Complete reliable serial receive, status publication, sequence-aware command delivery, completion acknowledgement, ordered writes, and reconnect handling.

**Architecture:** Keep protocol code dependency-free and put Boost.Asio ownership behind `LowerMachineSerial`. Keep `LowerMachineNode` limited to ROS message translation.

**Tech Stack:** C++14, ROS2 Dashing, Boost.Asio, GTest, Python unittest with cppyy.

1. Add command encoder and package contract tests.
2. Implement the 21-byte command encoder with a two-byte sequence.
3. Implement mixed 23-byte status and 8-byte ACK buffering.
4. Implement A1 parsing, 200 ms timeout, three retries, and strict matching.
5. Implement completion sequence parsing, business deduplication, and A2 encoding.
6. Keep one asynchronous serial owner and one ordered write queue for commands and ACKs.
7. Connect transport, parser, publisher, command subscriber, and retry timer in the ROS2 node.
8. Add launch/configuration wiring and run all locally available tests.

**Current status:** Steps 1-8 are implemented and pass local dependency-free tests. Ubuntu 18.04 ROS2 Dashing compilation, pseudo-terminal ACK loss tests, lower-machine firmware integration, and real-vehicle verification remain before Stage 2 acceptance.
