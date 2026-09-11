# Multi-goal Task UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a panel-driven one-to-three-point navigation task workflow with atomic submission, ordered execution, stop confirmation, authoritative per-point results, and fail-stop behavior.

**Architecture:** Qt owns only an editable `TaskDraft`; `SandboxDisplayNode` converts the confirmed draft into one map-frame `PoseArray`. `NavigationTaskQueue` remains the execution authority and publishes one transient-local typed `NavigationTaskState` snapshot consumed through the existing thread-safe `DisplayState` revision mechanism.

**Tech Stack:** ROS 2 Humble, Python 3.10, `rclpy`, Nav2 `NavigateToPose`, ROSIDL messages, PyQt5, pytest, colcon.

**Spec:** `docs/superpowers/specs/2026-09-11-multi-goal-task-ui-design.md`

## Global Constraints

- Support exactly one to three `map`-frame goals in click order.
- Submit at most one active `NavigateToPose` goal.
- Advance only after Nav2 success and fresh velocity has remained below the configured threshold for 0.5 seconds.
- Any failure, cancellation, localization fault, or stop timeout terminates the task and prevents every later goal from being dispatched.
- Automatic retry is disabled; every point's `retry_count` is zero.
- Preserve terminal state and later `NOT_EXECUTED` points through a reliable transient-local snapshot.
- Keep `/goal_pose` and existing `/navigation_task/*` compatibility topics.
- ROS callbacks must never manipulate Qt widgets directly.
- Work and commits remain local. Do not push any branch or commit to GitHub without explicit user approval after all verification results are reported.

## File Structure

- Create `src/landerpi_msgs/msg/NavigationPointState.msg`: typed state for one target.
- Create `src/landerpi_msgs/msg/NavigationTaskState.msg`: atomic task snapshot.
- Modify `src/landerpi_msgs/CMakeLists.txt`: generate both messages.
- Modify `src/landerpi_msgs/package.xml`: declare ROSIDL and message dependencies.
- Modify `src/landerpi_task_manager/landerpi_task_manager/task_queue.py`: per-point records, measurements, terminal preservation, and state publisher.
- Create `src/landerpi_task_manager/test/test_task_queue.py`: pure and callback-level queue state tests.
- Modify `src/landerpi_task_manager/package.xml`: depend on `landerpi_msgs`.
- Create `src/landerpi_sandbox_display/landerpi_sandbox_display/task_draft.py`: pure goal drafting and yaw logic.
- Create `src/landerpi_sandbox_display/test/test_task_draft.py`: draft unit tests.
- Modify `src/landerpi_sandbox_display/landerpi_sandbox_display/sandbox_display_node.py`: task/cancel publishers and typed state subscriber.
- Modify `src/landerpi_sandbox_display/landerpi_sandbox_display/ui_state.py`: task snapshot revision.
- Modify `src/landerpi_sandbox_display/landerpi_sandbox_display/map_widget.py`: numbered draft/result markers.
- Modify `src/landerpi_sandbox_display/landerpi_sandbox_display/main_window.py`: modes, task table, controls, and active-state lock.
- Modify display tests: ROS adapter, state, map rendering, and window interaction coverage.
- Modify display and navigation package manifests to depend on `landerpi_msgs` where required.

---

### Task 1: Generate typed task-state messages

**Files:**
- Create: `src/landerpi_msgs/msg/NavigationPointState.msg`
- Create: `src/landerpi_msgs/msg/NavigationTaskState.msg`
- Modify: `src/landerpi_msgs/CMakeLists.txt`
- Modify: `src/landerpi_msgs/package.xml`

**Interfaces:**
- Produces: `landerpi_msgs.msg.NavigationPointState`
- Produces: `landerpi_msgs.msg.NavigationTaskState`

- [ ] **Step 1: Add the point message**

```text
uint8 PENDING=0
uint8 PLANNING=1
uint8 NAVIGATING=2
uint8 WAITING_FOR_STOP=3
uint8 SUCCEEDED=4
uint8 FAILED=5
uint8 CANCELLED=6
uint8 NOT_EXECUTED=7
uint8 index
geometry_msgs/Pose target
uint8 state
bool has_arrival_error
float32 arrival_error_m
float32 elapsed_time_s
uint8 retry_count
string detail
```

- [ ] **Step 2: Add the task snapshot message**

```text
uint8 IDLE=0
uint8 ACTIVE=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 CANCELLED=4
uint8 REJECTED=5
std_msgs/Header header
uint32 task_id
bool active
uint8 current_index
uint8 total_points
uint8 state
float32 elapsed_time_s
string detail
landerpi_msgs/NavigationPointState[] points
```

- [ ] **Step 3: Configure ROSIDL generation**

```cmake
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationPointState.msg"
  "msg/NavigationTaskState.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 4: Add manifest dependencies**

Add `geometry_msgs`, `std_msgs`, `rosidl_default_generators`,
`rosidl_default_runtime`, and membership in `rosidl_interface_packages`.

- [ ] **Step 5: Build and inspect the interfaces**

Run:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select landerpi_msgs
source install/setup.bash
ros2 interface show landerpi_msgs/msg/NavigationPointState
ros2 interface show landerpi_msgs/msg/NavigationTaskState
```

Expected: build exits zero and both message definitions contain every field and constant above.

- [ ] **Step 6: Commit locally**

```bash
git add src/landerpi_msgs
git commit -m "feat(msgs): add structured navigation task state"
```

Do not push.

### Task 2: Make the task queue publish authoritative point records

**Files:**
- Modify: `src/landerpi_task_manager/landerpi_task_manager/task_queue.py`
- Create: `src/landerpi_task_manager/test/test_task_queue.py`
- Modify: `src/landerpi_task_manager/package.xml`

**Interfaces:**
- Consumes: `NavigationPointState`, `NavigationTaskState`
- Produces: transient-local `/navigation_task/state`
- Produces: `_snapshot_message(now_ns: int) -> NavigationTaskState`
- Produces: `_set_point_state(index: int, state: int, detail: str) -> None`
- Produces: `_finish_point(index: int, state: int, detail: str, now_ns: int) -> None`

- [ ] **Step 1: Write failing record/state tests**

Create pure record helpers that can be exercised without spinning ROS:

```python
def test_success_records_error_time_and_zero_retries():
    record = PointRecord(index=1, target=make_pose())
    record.start(1_000_000_000)
    record.finish(
        state=NavigationPointState.SUCCEEDED,
        detail='stopped',
        now_ns=3_500_000_000,
        arrival_error_m=0.08,
    )
    assert record.elapsed_time_s == 2.5
    assert record.arrival_error_m == 0.08
    assert record.has_arrival_error is True
    assert record.retry_count == 0


def test_failure_marks_later_points_not_executed():
    records = make_records(3)
    finalize_records(records, failed_index=1, failed_state=FAILED,
                     reason='AMCL pose timed out', now_ns=2_000_000_000)
    assert [record.state for record in records] == [
        SUCCEEDED, FAILED, NOT_EXECUTED,
    ]
```

- [ ] **Step 2: Run the focused tests and verify they fail**

Run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
pytest -q src/landerpi_task_manager/test/test_task_queue.py
```

Expected: collection or assertions fail because the record helpers do not exist.

- [ ] **Step 3: Add a focused `PointRecord` dataclass**

Implement fields for target, state, detail, start time, elapsed time, optional
arrival error, and retry count. Keep message conversion in `to_message()` and
normalize unavailable error to `0.0` with `has_arrival_error=False`.

- [ ] **Step 4: Publish the transient-local task snapshot**

Add:

```python
self._task_state = self.create_publisher(
    NavigationTaskState,
    '/navigation_task/state',
    latched,
)
```

Create records when an idle submission arrives. Publish on rejection,
acceptance, planning, navigating, waiting-for-stop, stop confirmation,
advancement, cancellation, failure, and cleanup. Preserve the terminal index
and records instead of resetting the observable result to an empty task.

- [ ] **Step 5: Record error at confirmed stop**

Keep the latest valid map-frame AMCL XY coordinates. In `_advance_or_finish`,
calculate:

```python
error = math.hypot(
    current_target.position.x - latest_amcl_x,
    current_target.position.y - latest_amcl_y,
)
```

Finish the current point as `SUCCEEDED` only after fresh zero velocity has
remained continuous for the configured settle duration.

- [ ] **Step 6: Add active-state 5 Hz snapshots**

Reuse `_monitor` or add a 0.2-second timer. While claimed, update total/current
elapsed values and publish one complete snapshot. Do not generate status log
lines for these timer-only refreshes.

- [ ] **Step 7: Cover queue sequencing and all terminal paths**

Add tests proving:

```python
assert sent_goal_indices == [0]             # before first result
assert sent_goal_indices == [0]             # before 0.5 s zero velocity
assert sent_goal_indices == [0, 1]          # after confirmed stop
assert sent_goal_indices == [0, 1]          # point 2 failure; point 3 absent
```

Also cover cancellation, stale AMCL, excessive covariance, stop timeout,
server unavailable, and transient-local publisher QoS.

- [ ] **Step 8: Run task-manager tests**

Run:

```bash
pytest -q src/landerpi_task_manager/test
```

Expected: all tests pass.

- [ ] **Step 9: Commit locally**

```bash
git add src/landerpi_task_manager
git commit -m "feat(task): publish per-point navigation results"
```

Do not push.

### Task 3: Add the pure front-end task draft model

**Files:**
- Create: `src/landerpi_sandbox_display/landerpi_sandbox_display/task_draft.py`
- Create: `src/landerpi_sandbox_display/test/test_task_draft.py`

**Interfaces:**
- Produces: `DraftGoal(x: float, y: float, yaw: float, yaw_manual: bool)`
- Produces: `TaskDraft(max_goals: int = 3)`
- Produces: `TaskDraft.add(x: float, y: float, robot_yaw: float | None) -> bool`
- Produces: `TaskDraft.set_yaw(index: int, yaw: float) -> None`
- Produces: `TaskDraft.undo() -> None`, `clear() -> None`, `goals() -> tuple[DraftGoal, ...]`
- Produces: `yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]`

- [ ] **Step 1: Write draft behavior tests**

```python
def test_draft_keeps_click_order_and_rejects_fourth():
    draft = TaskDraft(max_goals=3)
    assert draft.add(0.0, 0.0, robot_yaw=0.2)
    assert draft.add(1.0, 0.0, robot_yaw=0.2)
    assert draft.add(1.0, 1.0, robot_yaw=0.2)
    assert not draft.add(2.0, 2.0, robot_yaw=0.2)
    assert [(g.x, g.y) for g in draft.goals()] == [
        (0.0, 0.0), (1.0, 0.0), (1.0, 1.0),
    ]


def test_manual_yaw_is_not_overwritten_by_later_click():
    draft = TaskDraft()
    draft.add(0.0, 0.0, robot_yaw=0.0)
    draft.set_yaw(0, 1.25)
    draft.add(1.0, 1.0, robot_yaw=0.0)
    assert draft.goals()[0].yaw == 1.25
```

- [ ] **Step 2: Verify the tests fail**

Run: `pytest -q src/landerpi_sandbox_display/test/test_task_draft.py`

Expected: fail because `task_draft` is absent.

- [ ] **Step 3: Implement minimal draft logic**

Use `math.atan2(next_y - y, next_x - x)` to update only automatic yaw values.
For one point use the available robot yaw, otherwise zero. Make `goals()` return
copies or immutable dataclass values so publication cannot mutate the draft.

- [ ] **Step 4: Test undo, clear, non-finite rejection, auto final yaw, and quaternion normalization**

Quaternion assertions must verify `x=y=0` and
`sqrt(z*z + w*w) == pytest.approx(1.0)`.

- [ ] **Step 5: Run draft tests**

Run: `pytest -q src/landerpi_sandbox_display/test/test_task_draft.py`

Expected: all tests pass.

- [ ] **Step 6: Commit locally**

```bash
git add src/landerpi_sandbox_display/landerpi_sandbox_display/task_draft.py \
        src/landerpi_sandbox_display/test/test_task_draft.py
git commit -m "feat(display): add multi-goal task draft"
```

Do not push.

### Task 4: Connect the display ROS adapter to task commands and state

**Files:**
- Modify: `src/landerpi_sandbox_display/landerpi_sandbox_display/sandbox_display_node.py`
- Modify: `src/landerpi_sandbox_display/landerpi_sandbox_display/ui_state.py`
- Modify: `src/landerpi_sandbox_display/package.xml`
- Modify: `src/landerpi_sandbox_display/test/test_robot_tf_pose.py`
- Modify: `src/landerpi_sandbox_display/test/test_ui_state.py`

**Interfaces:**
- Consumes: tuple of `DraftGoal`
- Produces: `publish_navigation_task(goals) -> bool`
- Produces: `cancel_navigation_task() -> None`
- Consumes: `/navigation_task/state`
- Produces: `DisplayState.update_navigation_task_state(message)`

- [ ] **Step 1: Write failing publisher and state tests**

Use a fake publisher and clock to assert one message:

```python
assert message.header.frame_id == 'map'
assert len(message.poses) == 3
assert [pose.position.x for pose in message.poses] == [1.0, 2.0, 3.0]
assert cancel_publisher.messages == [Empty()]
```

Add a `DisplayState` test asserting value and revision under
`navigation_task_state`.

- [ ] **Step 2: Verify focused tests fail**

Run the two exact test modules and confirm missing methods or publishers.

- [ ] **Step 3: Add ROS publishers/subscriber**

Create reliable publishers for `/navigation_task/goals` and
`/navigation_task/cancel`; subscribe to `/navigation_task/state` with reliable,
transient-local depth 1. Convert each draft yaw through
`yaw_to_quaternion` and publish only when the count is one to three and all
values are finite.

- [ ] **Step 4: Wire Qt callbacks without direct ROS access**

Add to `MainWindow` callback setters, then connect them in `main()`:

```python
window.set_task_publish_callback(node.publish_navigation_task)
window.set_task_cancel_callback(node.cancel_navigation_task)
```

- [ ] **Step 5: Run focused tests**

Run:

```bash
pytest -q \
  src/landerpi_sandbox_display/test/test_robot_tf_pose.py \
  src/landerpi_sandbox_display/test/test_ui_state.py
```

Expected: all tests pass.

- [ ] **Step 6: Commit locally**

```bash
git add src/landerpi_sandbox_display
git commit -m "feat(display): connect navigation task ROS interfaces"
```

Do not push.

### Task 5: Render numbered draft and execution markers

**Files:**
- Modify: `src/landerpi_sandbox_display/landerpi_sandbox_display/map_widget.py`
- Modify: `src/landerpi_sandbox_display/test/test_map_widget.py`

**Interfaces:**
- Produces: `set_task_markers(markers: list[TaskMarker]) -> None`
- `TaskMarker` contains index, x, y, yaw, and state.

- [ ] **Step 1: Write failing marker rendering tests**

Test that setter storage requests repaint and that a rendered image contains
visible numbered markers at transformed coordinates. Check shape/text presence
for current, succeeded, failed, and not-executed states rather than asserting
only color values.

- [ ] **Step 2: Verify marker tests fail**

Run: `pytest -q src/landerpi_sandbox_display/test/test_map_widget.py`

Expected: failure because marker state and rendering are absent.

- [ ] **Step 3: Implement marker rendering**

Draw the quiet dashed connector first, then markers. Use a numbered circle for
pending/success, a numbered diamond for current/waiting, and a numbered cross
for failed/not-executed so meaning does not depend on color. Convert every
position with the existing `world_to_widget` method.

- [ ] **Step 4: Run marker and coordinate tests**

Run:

```bash
pytest -q \
  src/landerpi_sandbox_display/test/test_map_widget.py \
  src/landerpi_sandbox_display/test/test_coordinate_transform.py
```

Expected: all tests pass with existing map/path rendering unchanged.

- [ ] **Step 5: Commit locally**

```bash
git add src/landerpi_sandbox_display/landerpi_sandbox_display/map_widget.py \
        src/landerpi_sandbox_display/test/test_map_widget.py
git commit -m "feat(display): render ordered task markers"
```

Do not push.

### Task 6: Add panel task-building and result interaction

**Files:**
- Modify: `src/landerpi_sandbox_display/landerpi_sandbox_display/main_window.py`
- Modify: `src/landerpi_sandbox_display/test/test_main_window.py`

**Interfaces:**
- Consumes: `TaskDraft`
- Consumes: `NavigationTaskState`
- Produces: task publish and cancel callback invocations

- [ ] **Step 1: Write failing window interaction tests**

Cover mode switching, three ordered clicks, fourth-click rejection, undo,
clear, yaw editing, one publish on submit, one cancel per click, result rows,
and active-state locking. Include:

```python
state.update_navigation_task_state(active_snapshot(current_index=2))
window.refresh_from_state(state)
assert not window.submit_task_button.isEnabled()
assert not window.single_goal_mode_button.isEnabled()
assert window.cancel_task_button.isEnabled()
```

- [ ] **Step 2: Verify the focused window tests fail**

Run only the new test cases and confirm the missing widgets/callbacks.

- [ ] **Step 3: Add mode and task controls**

Add explicit single/multi mode controls, `Undo last`, `Clear`, `Submit task`,
and `Cancel task`. Use a compact three-row `QTableWidget` with columns for point,
coordinates/yaw, state, arrival error, elapsed time, and retries. Keep the map
as the dominant surface.

- [ ] **Step 4: Implement active-state arbitration**

While a multi-goal snapshot has `active=true`, ignore map edits and disable
single-goal submission. Terminal snapshots unlock editing but remain displayed.
Ignore stale single-goal status/error/time updates in the multi-goal result
area.

- [ ] **Step 5: Run main-window tests**

Run: `pytest -q src/landerpi_sandbox_display/test/test_main_window.py`

Expected: all tests pass, including the existing single-goal behavior.

- [ ] **Step 6: Commit locally**

```bash
git add src/landerpi_sandbox_display/landerpi_sandbox_display/main_window.py \
        src/landerpi_sandbox_display/test/test_main_window.py
git commit -m "feat(display): add multi-goal navigation panel"
```

Do not push.

### Task 7: Build, integrate, and verify the complete local workflow

**Files:**
- Modify only if required by discovered build wiring errors: package manifests,
  CMake install rules, and launch dependencies already listed above.
- Test: all affected package tests and existing acceptance checks.

**Interfaces:**
- Consumes every interface produced by Tasks 1-6.
- Produces a locally verified integration branch; no remote update.

- [ ] **Step 1: Run all affected Python tests**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
pytest -q \
  src/landerpi_task_manager/test \
  src/landerpi_sandbox_display/test
```

Expected: zero failures and zero collection errors.

- [ ] **Step 2: Build all affected ROS packages**

```bash
colcon build --symlink-install --packages-select \
  landerpi_msgs \
  landerpi_task_manager \
  landerpi_navigation \
  landerpi_sandbox_display
```

Expected: all four packages finish successfully.

- [ ] **Step 3: Run package tests and inspect results**

```bash
colcon test --packages-select \
  landerpi_msgs \
  landerpi_task_manager \
  landerpi_navigation \
  landerpi_sandbox_display
colcon test-result --verbose
```

Expected: zero failed tests.

- [ ] **Step 4: Run static repository checks**

```bash
git diff --check
python3 -m compileall -q \
  src/landerpi_task_manager/landerpi_task_manager \
  src/landerpi_sandbox_display/landerpi_sandbox_display
```

Expected: both commands exit zero.

- [ ] **Step 5: Perform the local three-point integration scenario**

Start the existing simulation/navigation launch, set a valid AMCL initial pose,
submit three panel points, and record evidence that:

- exactly one Nav2 action is active;
- point 2 is not dispatched before point 1 has 0.5 seconds of confirmed zero velocity;
- the terminal snapshot contains all three point results;
- cancellation or a forced point-2 failure prevents point 3.

- [ ] **Step 6: Report results and stop for approval**

Provide the user with the branch name, local commit list, changed-file summary,
pytest count, colcon result, and any unverified hardware-only checks. Do not push,
open a pull request, merge to a shared branch, or delete the source stash.
