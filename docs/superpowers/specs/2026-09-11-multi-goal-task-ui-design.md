# Multi-goal Task UI and Structured State Design

## Context

The latest navigation work lives on `feature/navigation` and provides a
one-to-three-goal queue. The latest display work consists of
`feature/display` plus `stash@{0}`, which contains the Week 4 navigation
status, arrival status, task time, and `DisplayState` synchronization work.
These sources are not yet present together in one branch.

The existing queue already enforces the primary safety rule:

> Validate, dispatch one point, wait for Nav2 success, prove that velocity has
> remained zero for 0.5 seconds, then dispatch the next point.

The missing pieces are the panel-side task builder, authoritative per-point
results, and a reliable structured state channel between the task manager and
the display.

## Goals

- Let the operator select one to three ordered goals in the LanderPi panel.
- Submit all selected goals atomically as one `geometry_msgs/PoseArray`.
- Never send the second goal before the first has succeeded and the vehicle has
  been confirmed stopped.
- Stop the complete task after any navigation, cancellation, localization, or
  stop-confirmation failure.
- Preserve and display the failure point and the reason; never represent later
  points as completed.
- Record authoritative per-point state, arrival error, elapsed time, and retry
  count in the task manager.
- Restore the latest task result if the display starts or reconnects after the
  task has already changed state.
- Keep the existing single-goal `/goal_pose` workflow compatible.

## Non-goals

- More than three goals.
- Automatic retries. `retry_count` remains zero in this version.
- Skipping a failed point and continuing with later points.
- Resuming a failed or cancelled queue.
- Replacing the existing command topics with a new custom ROS action.
- Using RViz as the multi-goal task entry point.

## Selected Architecture

The implementation keeps the existing topic-based command interface and adds
one typed, latched task-state snapshot.

```text
Map clicks
    -> MainWindow task draft
    -> SandboxDisplayNode ROS adapter
    -> /navigation_task/goals (PoseArray)
    -> NavigationTaskQueue
    -> one NavigateToPose at a time
    -> confirmed zero velocity for 0.5 s
    -> next point or terminal state

NavigationTaskQueue
    -> /navigation_task/state (NavigationTaskState)
    -> SandboxDisplayNode
    -> DisplayState revision snapshot
    -> MainWindow and MapWidget
```

No web server or additional process is required between Qt and ROS. Qt passes
an immutable draft to `SandboxDisplayNode`; that node owns ROS message
construction and publication. `NavigationTaskQueue` remains the sole authority
for execution state and results.

## Command Interfaces

### Submit a task

- Topic: `/navigation_task/goals`
- Type: `geometry_msgs/msg/PoseArray`
- QoS: reliable, volatile, depth 10
- `header.frame_id`: exactly `map`
- `header.stamp`: set by `SandboxDisplayNode` when the operator confirms
- `poses`: one to three `geometry_msgs/msg/Pose` values in click order
- Every position value must be finite.
- Every orientation quaternion must contain finite values and have non-zero
  norm. The publisher normalizes it before sending.

`PoseArray` contains `Pose` elements, not `PoseStamped` elements. The common
frame and timestamp belong to the array header.

### Cancel a task

- Topic: `/navigation_task/cancel`
- Type: `std_msgs/msg/Empty`
- QoS: reliable, volatile, depth 10

The panel publishes one cancel message per operator action and immediately
disables the cancel button until a newer task-state snapshot arrives.

### Existing compatibility topics

The queue continues publishing:

- `/navigation_task/status`
- `/navigation_task/active`
- `/navigation_task/current_index`

These remain useful for command-line inspection and existing consumers. The
panel does not parse `/navigation_task/status` to reconstruct per-point state;
free-form text and independently delivered topics cannot provide an atomic,
reconnect-safe view.

## Structured State Interface

The placeholder `landerpi_msgs` package will define two messages.

### `NavigationPointState.msg`

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

`index` is one-based. `arrival_error_m` is meaningful only when
`has_arrival_error` is true. `retry_count` is present for reporting and future
policy changes but is always zero while the fail-stop policy has no retry.

### `NavigationTaskState.msg`

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

- Topic: `/navigation_task/state`
- Type: `landerpi_msgs/msg/NavigationTaskState`
- QoS: reliable, transient-local, depth 1
- `header.frame_id`: `map`
- `current_index`: one-based while a point is selected; zero only for an idle
  task with no retained result
- Terminal snapshots retain the last executed or failed index.
- A complete snapshot is published after every externally visible transition.
- While a task is active, the queue republishes the snapshot at 5 Hz so total
  and current-point elapsed time remain authoritative and visibly advance.

Using one snapshot avoids cross-topic ordering races such as `active=false`
arriving before the final human-readable status.

## Backend State and Measurement Rules

When a valid task is accepted, the queue creates all point records as
`PENDING`. It then follows this sequence for each point:

```text
PENDING
  -> PLANNING
  -> NAVIGATING
  -> WAITING_FOR_STOP
  -> SUCCEEDED
```

The queue records the point start time when it enters `PLANNING`. Time uses the
ROS node clock so simulation time and wall-clock operation behave consistently.
Elapsed time ends when the point becomes terminal or when stop confirmation
promotes it to `SUCCEEDED`.

When the queue is idle, each non-conflicting submission attempt receives a new
`task_id` before validation. A rejected request therefore produces a retained
`REJECTED` snapshot with its own identifier and reason. A competing request
received while another task is active does not replace the active task
snapshot; it is rejected only through the compatibility status channel.

The single-goal bridge and multi-goal queue submit to the internal
`/navigation_command` action. A dedicated arbiter is the only client allowed
to submit to Nav2's `/navigate_to_pose` action and accepts only one unresolved
command at a time. Before exposing its internal action after startup or restart,
the arbiter sends Nav2 a cancel-all request so an orphaned command cannot
overlap a new one. Unknown action outcomes keep the arbiter fail-closed. The
two transient-local `active` topics remain useful status signals but are not
the authoritative mutual-exclusion mechanism.

Arrival error is the map-frame XY Euclidean distance between the target and the
latest healthy AMCL pose. It is captured when continuous zero velocity has
been confirmed, not merely when Nav2 first reports success. If no healthy pose
is available at that moment, the task fails localization validation and no
arrival error is claimed.

The queue publishes a new snapshot on acceptance, dispatch, Nav2 acceptance,
Nav2 success, stop confirmation, point advancement, cancellation, and every
terminal failure.

## Failure Policy

Any of the following terminates the whole task:

- Invalid count, frame, position, or quaternion at submission.
- AMCL pose timeout, wrong frame, non-finite covariance, or covariance above
  the configured thresholds.
- NavigateToPose server unavailable, goal rejected, transport error, cancelled
  result, or failed result.
- Stop velocity missing, stale, above threshold, or not continuously zero for
  0.5 seconds before the stop timeout.
- Explicit operator cancellation.

For a failure at point `N`:

- Point `N` becomes `FAILED` or `CANCELLED` with the exact reason.
- Earlier successful points remain `SUCCEEDED`.
- Every later point becomes `NOT_EXECUTED` with detail
  `task stopped after point N`.
- The pending internal queue is cleared.
- No automatic retry or later-point dispatch occurs.
- The terminal snapshot is retained with `active=false`.

## Panel Interaction

The navigation panel gains explicit `Single goal` and `Multi-goal task` modes.
The current single-goal behavior remains unchanged outside an active task.

In multi-goal mode:

1. Each valid map click appends one draft point, up to three.
2. `MapWidget` renders numbered markers and a quiet dashed connector in order.
3. A task list shows index, X, Y, yaw, execution state, error, elapsed time, and
   retry count.
4. Before submission the operator can undo the last point, clear the draft, or
   edit a point's yaw.
5. `Submit task` is enabled only for one to three valid points while no task is
   active.
6. Confirmation passes one immutable goal list to `SandboxDisplayNode`, which
   publishes one `PoseArray`.
7. While active, map editing, submission, and the single-goal action are locked.
   `Cancel task` remains available.
8. On completion or failure, editing is unlocked but the result markers and
   table remain until the operator explicitly starts a new draft or clears it.

The backend still rejects competing commands. UI locking is guidance, not the
only concurrency protection.

## Goal Orientation

Map clicks select positions. The draft computes an initial yaw without hiding
the value from the operator:

- When point `N+1` is added, an unedited point `N` faces point `N+1`.
- The final point inherits the preceding segment direction.
- A one-point task uses the current robot yaw when available, otherwise zero.
- The list always displays yaw and allows an explicit override before submit.
- Once a yaw is edited manually, later clicks do not overwrite it.

Before publication, yaw is converted to a normalized planar quaternion with
`x=0`, `y=0`, `z=sin(yaw/2)`, and `w=cos(yaw/2)`.

## Display State

`SandboxDisplayNode` subscribes to `/navigation_task/state` and stores the
latest message through `DisplayState.update_navigation_task_state`. The Qt
timer continues using the existing revision-based snapshot mechanism; ROS
callbacks never manipulate widgets directly.

For multi-goal tasks, the task snapshot is authoritative for task status,
current point, arrival error, task/point time, and terminal result. Existing
`/navigation_status`, `/arrival_status`, and `/position_error` remain the
single-goal data source. This prevents stale single-goal telemetry from
overwriting an active multi-goal display.

## Expected Code Boundaries

- `landerpi_msgs`: add and build the two task-state messages.
- `landerpi_task_manager/task_queue.py`: own point records, measurements,
  failure finalization, and state publication.
- `landerpi_sandbox_display/task_draft.py`: pure draft ordering, limit,
  orientation, editing, and validation logic.
- `landerpi_sandbox_display/sandbox_display_node.py`: publish task/cancel
  commands and subscribe to structured state.
- `landerpi_sandbox_display/ui_state.py`: add the task-state revision value.
- `landerpi_sandbox_display/main_window.py`: mode, task list, controls, and
  lock/unlock behavior.
- `landerpi_sandbox_display/map_widget.py`: numbered draft and execution
  markers.

The pure draft model keeps coordinate and orientation logic independently
testable and avoids growing `MainWindow` into a mixed UI/domain object.

## Test Strategy

### Unit tests

- Draft accepts one to three points in order and rejects a fourth.
- Undo, clear, yaw auto-fill, manual yaw preservation, and quaternion
  normalization.
- Publisher creates exactly one map-frame `PoseArray` in draft order.
- Submit and single-goal controls lock while `active=true` and unlock after a
  terminal snapshot.
- Map markers display pending, current, waiting-for-stop, succeeded, failed,
  and not-executed states without relying on color alone.
- `DisplayState` retains only the latest atomic task snapshot.
- Queue dispatches one Nav2 goal at a time.
- A successful Nav2 result does not advance before 0.5 seconds of fresh zero
  velocity.
- Every failure source clears pending execution and retains the failure point.
- Arrival error and elapsed time are captured at the defined transitions.
- Retry count remains zero and no retry is submitted.
- Cancellation marks later points `NOT_EXECUTED`.

### Integration tests

- Three goals produce exactly three ordered NavigateToPose requests.
- No request overlaps the previous goal or its stop-confirmation phase.
- Failure at point 2 produces point states
  `[SUCCEEDED, FAILED, NOT_EXECUTED]` and never submits point 3.
- Single-goal `/goal_pose` is rejected while the queue owns navigation.
- Simultaneous single-goal and multi-goal submissions produce only one Nav2
  action request.
- A display started after completion receives the terminal state snapshot.
- Simulation-time and real-time launches calculate elapsed time correctly.

### Real-vehicle acceptance

- Select three visibly distinct free-space points from the panel.
- Verify marker order and yaw before submission.
- Verify physical stop at each point before the next motion begins.
- Obstruct point 2 and confirm point 3 is never attempted.
- Cancel during point 2 and confirm no later motion occurs.
- Interrupt or invalidate AMCL and confirm immediate terminal failure.
- Compare each recorded arrival error with the final map-frame AMCL pose.

## Integration Preconditions

Before implementation, the Week 4 display stash must be preserved as a normal
commit, and implementation must occur on an integration branch that contains
both the latest display work and `feature/navigation`. The design must not be
implemented directly on `dev`, and the stash must not be dropped until its
commit is independently verified.

## Acceptance Criteria

The feature is complete only when one panel submission creates one ordered task,
Nav2 receives at most one active goal, every transition is visible through the
typed task snapshot, the vehicle is proven stopped before advancement, and all
failure paths retain the failed point while preventing every later goal from
being dispatched.
