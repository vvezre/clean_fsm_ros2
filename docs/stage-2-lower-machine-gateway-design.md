# Stage 2 Lower-Machine Gateway Design

## Scope

Complete the lower-machine gateway without moving mission logic into the hardware package. The gateway owns the serial device, distinguishes fixed 23-byte status frames from 8-byte ACK frames, publishes parsed status, and serializes final vehicle commands into the approved 21-byte sequence-aware format.

## Components

- `FrameBuffer`: preserves partial reads, distinguishes 23-byte status and 8-byte ACK frames, extracts sticky packets, removes noise, and resynchronizes after an invalid fixed tail.
- `StatusFrameParser`: validates and decodes one complete 23-byte status frame.
- `CommandEncoder`: deterministically creates one 21-byte command frame with a big-endian sequence. Brake overrides and clears every motion output.
- `AckCodec`: parses A1 command acknowledgements and encodes A2 completion acknowledgements.
- `CommandLifecycle`: matches A1 by sequence and command type, retries the exact original frame after timeout, tracks recent commands, and deduplicates completion by sequence and event type.
- `LowerMachineSerial`: the only owner of the serial port. It runs one Boost.Asio event thread, one asynchronous read chain, and one ordered write queue.
- `LowerMachineNode`: maps ROS parameters and messages to the protocol classes and publishes connection/status diagnostics.

## Data Flow

Receive: serial bytes -> append -> repeatedly pop -> dispatch status or ACK. Status frames are parsed and published to `/hardware/status`; A1 frames update command delivery state.

Send command: `/control/final_cmd` -> allocate sequence -> encode 21 bytes -> track -> enqueue -> wait for matching A1.

Send completion confirmation: 23-byte status with completion flag and known command sequence -> deduplicate business completion -> send A2 through the same ordered write queue. A duplicate completion is not published as a new business event, but A2 is sent again so a lost A2 can recover.

Commands are no longer sent five times unconditionally. The first frame is sent once; if no matching A1 arrives within 200 ms, the identical frame and sequence are retried up to three times.

## Failure Behavior

Open and read/write failures close the port, clear queued commands, publish disconnected state, and retry after one second. Clearing the queue prevents stale movement commands from being transmitted after a reconnect.

An unmatched or invalid A1 cannot clear a pending command. A completion event with an unknown sequence is published as ordinary status but is not acknowledged or allowed to advance mission logic. ACK frames are never acknowledged again.

## Verification

Local runtime tests execute the dependency-free protocol C++ through cppyy and cover byte layout, checksum, mixed-frame extraction, retry, A1 matching, completion deduplication, and A2 encoding. ROS2 compilation, pseudo-terminal testing, and real-device integration remain Ubuntu 18.04 acceptance checks because the current Windows host has no ROS2 Dashing toolchain.
