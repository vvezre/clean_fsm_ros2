# Maintenance Store Bootstrap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Ensure a fresh installation creates the persistent maintenance genesis record before the release becomes active, without allowing the runtime node to repair missing or corrupt safety state.

**Architecture:** Add one `cleanbot_mission` command-line executable that delegates all state validation and atomic persistence to the existing `MaintenanceStore::initializeGenesis()`. Invoke it from `deployment/install/install.sh` as the unprivileged `cleanbot` service user after the candidate release is copied but before system files and `/opt/cleanbot/current` are switched.

**Tech Stack:** C++17, ROS2 Humble/ament CMake, Bash, Python `unittest`, GoogleTest, ARM64 `colcon`.

---

## File map

- Create `src/cleanbot_mission/src/maintenance_store_init.cpp`: minimal process entry point for genesis initialization and exit-code reporting.
- Modify `src/cleanbot_mission/CMakeLists.txt`: build and install `maintenance_store_init`.
- Modify `deployment/install/install.sh`: run the release-local initializer as `cleanbot` against the fixed production state path.
- Modify `tests/test_deployment_files.py`: enforce build, installation-order, user, executable-path, and state-path contracts.
- Reuse `src/cleanbot_mission/test/test_maintenance_store.cpp`: its existing tests already exercise missing-record creation, idempotence, active-record preservation, invalid-state rejection, and atomic-store behavior.
- Create only ignored verification helpers under `.tmp/` if needed for Raspberry Pi orchestration; do not add them to the release.

### Task 1: Add the failing deployment contract

**Files:**
- Modify: `tests/test_deployment_files.py`
- Inspect: `src/cleanbot_mission/CMakeLists.txt`
- Inspect: `deployment/install/install.sh`

- [ ] **Step 1: Add one focused failing test**

Append this method to `DeploymentFilesTest`:

```python
    def test_install_bootstraps_maintenance_store_before_activation(self):
        mission_cmake = self.read("src/cleanbot_mission/CMakeLists.txt")
        initializer = self.read(
            "src/cleanbot_mission/src/maintenance_store_init.cpp"
        )
        install = self.read("deployment/install/install.sh")

        self.assertIn(
            "add_executable(maintenance_store_init "
            "src/maintenance_store_init.cpp)",
            mission_cmake,
        )
        self.assertIn(
            "install(TARGETS mission_core mission_manager_node "
            "maintenance_store_init",
            mission_cmake,
        )
        self.assertIn("MaintenanceStore store(argv[1]);", initializer)
        self.assertIn("store.initializeGenesis()", initializer)

        init_command = (
            'runuser -u cleanbot -- "${maintenance_initializer}" '
            '"/var/lib/cleanbot/runtime/maintenance.lock"'
        )
        self.assertIn(init_command, install)
        self.assertLess(
            install.index(init_command),
            install.index(
                'mv -Tf -- "${temporary_link}" /opt/cleanbot/current'
            ),
        )
```

- [ ] **Step 2: Run the test and verify RED**

Run:

```powershell
python -m unittest tests.test_deployment_files.DeploymentFilesTest.test_install_bootstraps_maintenance_store_before_activation -v
```

Expected: `FAIL` because `src/cleanbot_mission/src/maintenance_store_init.cpp` does not exist.

- [ ] **Step 3: Commit the verified failing contract**

```powershell
git add tests/test_deployment_files.py
git commit -m "test: require maintenance store bootstrap"
```

### Task 2: Build and install the initializer

**Files:**
- Create: `src/cleanbot_mission/src/maintenance_store_init.cpp`
- Modify: `src/cleanbot_mission/CMakeLists.txt`
- Modify: `deployment/install/install.sh`
- Test: `tests/test_deployment_files.py`

- [ ] **Step 1: Add the minimal C++ entry point**

Create `src/cleanbot_mission/src/maintenance_store_init.cpp`:

```cpp
#include <iostream>

#include "cleanbot_mission/maintenance_store.hpp"

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "usage: maintenance_store_init STATE_PATH\n";
    return 2;
  }

  cleanbot::mission::MaintenanceStore store(argv[1]);
  const auto result = store.initializeGenesis();
  if (result.code != cleanbot::mission::MaintenanceStoreCode::kOk) {
    std::cerr << "maintenance store initialization failed: "
              << result.message << '\n';
    return 1;
  }

  std::cout << result.message << '\n';
  return 0;
}
```

- [ ] **Step 2: Add the executable to the package**

Add after `mission_manager_node` in `src/cleanbot_mission/CMakeLists.txt`:

```cmake
add_executable(maintenance_store_init src/maintenance_store_init.cpp)
target_link_libraries(maintenance_store_init mission_core)
```

Change the installation declaration to:

```cmake
install(TARGETS mission_core mission_manager_node maintenance_store_init
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 3: Invoke the candidate-release tool before activation**

In `deployment/install/install.sh`, immediately after the candidate release copy/reuse block and before installing system support files, add:

```bash
maintenance_initializer="${release_target}/install/cleanbot_mission/lib/cleanbot_mission/maintenance_store_init"
if [[ ! -x ${maintenance_initializer} ]]; then
  echo "maintenance store initializer is missing from release" >&2
  exit 1
fi
runuser -u cleanbot -- "${maintenance_initializer}" \
  "/var/lib/cleanbot/runtime/maintenance.lock"
```

This ordering ensures a corrupt or inaccessible state prevents activation of the candidate release.

- [ ] **Step 4: Run the focused contract and verify GREEN**

Run:

```powershell
python -m unittest tests.test_deployment_files.DeploymentFilesTest.test_install_bootstraps_maintenance_store_before_activation -v
```

Expected: `OK`.

- [ ] **Step 5: Run all deployment contracts**

Run:

```powershell
python -m unittest tests.test_deployment_files tests.test_deployment_updater -q
```

Expected: all deployment tests pass with no failures.

- [ ] **Step 6: Commit the implementation**

```powershell
git add src/cleanbot_mission/src/maintenance_store_init.cpp src/cleanbot_mission/CMakeLists.txt deployment/install/install.sh
git commit -m "fix: bootstrap persistent maintenance state"
```

### Task 3: Verify the complete local source state

**Files:**
- Verify: all tracked source and tests

- [ ] **Step 1: Run whitespace and worktree checks**

Run:

```powershell
git diff --check
git status --short
```

Expected: no whitespace errors; only intentional ignored `.tmp` verification files may exist outside `git status`.

- [ ] **Step 2: Run the complete dependency-independent suite**

Run:

```powershell
python -m unittest discover -s tests -q
```

Expected: at least the previous 338 tests plus the new contract pass; platform-specific tests may remain explicitly skipped.

- [ ] **Step 3: Confirm the branch history**

Run:

```powershell
git log -5 --oneline
```

Expected: the failing-test commit precedes the implementation commit, and both follow the signed deployment automation commits.

### Task 4: Rebuild and test the exact source on Raspberry Pi

**Files:**
- Verify remotely in: `/home/ubuntu/cleanbot_verify/$commit/source`, where
  `$commit` is assigned from the exact local `HEAD`
- Do not modify: `/home/ubuntu/clean_fsm_ros2`

- [ ] **Step 1: Create an exact tracked-source archive**

Run from the worktree:

```powershell
$commit = git rev-parse HEAD
git archive --format=tar.gz --output=".tmp/cleanbot-$commit.tar.gz" HEAD
Get-FileHash ".tmp/cleanbot-$commit.tar.gz" -Algorithm SHA256
```

Expected: archive creation succeeds and prints one SHA256 digest.

- [ ] **Step 2: Copy and verify the archive in a new isolated directory**

Use SSH/SCP with:

```text
-o KexAlgorithms=curve25519-sha256
-o BatchMode=yes
-o ConnectTimeout=10
```

Create only `/home/ubuntu/cleanbot_verify/$commit`, copy the archive there,
compare SHA256, and extract it to `source`. Copy the exact hexadecimal value
printed by `git rev-parse HEAD`; do not derive or shorten it.

- [ ] **Step 3: Run the official ARM64 verification path**

In the isolated source:

```bash
source /opt/ros/humble/setup.bash
python3 -m unittest tests.test_deployment_files tests.test_deployment_updater -q
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Expected: deployment contracts pass; all 11 packages build; `colcon test-result` reports zero errors and zero failures. `test_maintenance_store` must pass, proving initialization remains idempotent and does not overwrite invalid or active state.

### Task 5: Perform isolated stationary safety integration

**Files:**
- Create ignored helper: `.tmp/verify-$commit-static-integration.py`
- Create ignored wrapper if required: `.tmp/verify-$commit-static-integration.sh`
- Capture ignored local logs: `.tmp/pi-$commit-static-integration.out` and `.err`

- [ ] **Step 1: Prepare isolated runtime state**

Use a fresh ROS domain ID and paths under
`/home/ubuntu/cleanbot_verify/$commit/runtime` for the config database,
modeling database, checkpoints, ROS home, and ROS logs.

Because `mission_manager_node` intentionally fixes its safety-state path at
`/var/lib/cleanbot/runtime/maintenance.lock`, run the integration wrapper in a
private mount namespace:

```bash
sudo install -d -m 0755 /var/lib/cleanbot
sudo unshare --mount --propagation private \
  bash /home/ubuntu/cleanbot_verify/$commit/verify-static-integration.sh
```

Inside the wrapper, bind
`/home/ubuntu/cleanbot_verify/$commit/runtime/var-lib-cleanbot` onto
`/var/lib/cleanbot`, then invoke the newly built initializer against
`/var/lib/cleanbot/runtime/maintenance.lock`. The bind mount disappears when
the namespace exits; the wrapper must record whether it created the outer empty
mountpoint and remove only that empty directory afterward. Do not start
`lower_machine_node`, `rtk_node`, `tracking_node`, or
`command_arbiter_node`.

- [ ] **Step 2: Start only safe nodes**

Start these executables from the isolated install tree:

```text
cleanbot_config/config_manager_node
cleanbot_mission/mission_manager_node
cleanbot_modeling/modeling_manager_node
cleanbot_http/http_gateway_node
```

Use the central config service to set the isolated database/checkpoint paths and a nonproduction HTTP port before starting dependent nodes.

- [ ] **Step 3: Verify node and QoS behavior**

Assert all four nodes appear in the ROS graph. Create a late transient-local subscriber to `/vehicle/state` after `mission_manager_node` has already published; require a snapshot within two seconds.

- [ ] **Step 4: Verify safe Services and HTTP**

Call `/config/get`, `/mission/set_pause` with no active mission, and `/control/manual` with zero speeds plus `brake=true`. Require structured, bounded responses. Request `GET /api/v1/vehicle/state` and require HTTP 200 with `success=true`; request mission pause and require the downstream `NO_ACTIVE_MISSION` result rather than a hang.

- [ ] **Step 5: Verify Action acceptance, rejection, and timeout**

For `/mission/return_home`:

- send an invalid `(0, 0)` target and require goal rejection;
- send a valid coordinate without RTK/hardware publishers and require goal acceptance followed by a bounded aborted result with `RTK_NOT_READY` or the configured readiness-timeout code;
- enter maintenance mode while idle, send another valid goal, and require rejection; release the exact returned maintenance generation afterward.

At no time start a final-command consumer or hardware gateway, so no command can reach a device.

- [ ] **Step 6: Clean up and inspect logs**

Stop only PIDs created by the wrapper, unmount only the private bind mount, and
assert no cleanbot test process remains. After the namespace exits, verify that
the host `/var/lib/cleanbot` contains no test data; remove it with `rmdir` only
when this run created it and it remains empty. Inspect every node log for fatal
errors, crashes, and unbounded waits.

- [ ] **Step 7: Record the result**

The wrapper must print one final marker:

```text
STATIC_INTEGRATION_COMPLETE=$commit
```

Only treat the integration as passing when every assertion succeeds and the marker is present.

### Task 6: Push the verified branch

**Files:**
- Verify: committed feature branch

- [ ] **Step 1: Confirm clean tracked state**

Run:

```powershell
git status --short
```

Expected: empty output.

- [ ] **Step 2: Push the branch**

Run:

```powershell
git push origin deployment-automation
```

Expected: `origin/deployment-automation` advances to the verified implementation commit.

- [ ] **Step 3: Report remaining release gates**

Report that source implementation and Raspberry Pi stationary verification are complete, while merge to `main`, GitHub protected environment/signing secret, stable Release creation, production installation, and real-device/RTK testing remain separate user-approved operations.
