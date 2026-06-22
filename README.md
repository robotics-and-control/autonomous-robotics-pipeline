# Autonomous Mobile Manipulation with TIAGo

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-FF6C00?logo=gazebo&logoColor=white)
![Nav2](https://img.shields.io/badge/Nav2-AMCL_%2B_costmaps-1F6FEB)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Manipulation-0A7BBB)
![Python](https://img.shields.io/badge/Python-3-3776AB?logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-All_rights_reserved-555555)

A fully autonomous mobile-manipulation pipeline for the TIAGo robot, evaluated in a custom Gazebo world. Three sequential tasks share state and run end-to-end with no human intervention and no hardcoded positions. Every table, cube, and grasp pose is discovered at runtime from live sensor data.

## Highlights

- **Zero hardcoded poses.** Every table, cube, and grasp pose is discovered at runtime from ArUco markers and live sensor data; the place pose is derived from a relative transform measured at pick time, so it holds regardless of table or cube position.
- **Autonomous re-localization from an unknown start.** AMCL is driven to convergence by an active micro-exploration loop, with costmaps cleared each iteration to reject motion-induced phantom obstacles.
- **Runtime collision-scene management.** The target cube is removed from and re-inserted into the MoveIt 2 collision world around the grasp, with `link_attacher` providing stable contact in Gazebo.
- **Non-blocking mission control.** The ArUco-discovery state machine cancels and later resumes Nav2 goals on marker detection, with a watchdog that recovers from lost markers instead of deadlocking.
- **Timer-free map completion.** Mapping ends on a baseline-delta convergence test that accumulates slow sub-threshold growth, not a fixed timeout.

> **This repository is a public showcase.** The complete, runnable source code is kept in a private repository and is available on request.

---

## Demo

### Task 1: Autonomous SLAM Mapping
> Robot explores the unknown environment, builds a complete map, and saves it via a ROS 2 service when the map stops growing.

https://github.com/user-attachments/assets/9780a5ca-1464-4d82-b440-56e6e74aadda

### Task 2: Autonomous Re-localization and Table Discovery
> Robot starts from a random unknown pose, converges AMCL via micro-exploration, then discovers both pick and place tables using ArUco markers.

https://github.com/user-attachments/assets/6144b28e-200a-432e-b728-f8a36cedbbbd

### Task 3: Pick and Place (two cubes)
> Robot navigates to the pick table, grasps cube 63 then cube 582 using MoveIt 2, and carries each to the place table. Every grasp and place pose is recomputed at runtime.

https://github.com/user-attachments/assets/e7a6983c-0608-4b2e-90b1-8622a5fa6c4d

---

## System Architecture

```
Sensors: head RGB camera · laser scanner
Robot:   7-DOF arm + gripper · mobile base (TIAGo, PAL Robotics)

ROS 2 stacks used:
  slam_toolbox      explore_lite      Nav2 (AMCL + costmaps)
  aruco_ros         MoveIt 2          link_attacher

Custom nodes (this package):
  frontier_map_saver          Map convergence detection + save via service
  autonomous_localization     AMCL convergence driver via micro-exploration
  viewpoint_planner           Geometric wall-standoff viewpoint computation
  mission_executor            ArUco discovery state machine + Nav2 client
  pick_and_place              Full grasp-and-place state machine via MoveIt 2
  gripper_manager             Gripper action server wrapping pymoveit2
  home_config_arm             Arm homing via MoveIt 2 at task start
  set_nav2_params_task[2,3]   Runtime costmap parameter patching per task
```

Task-to-task state is persisted via YAML files (pick_frames, place_frames, robot_last_pose) so tasks can be launched independently and task 3 can resume after task 2 without re-running Gazebo.

---

## Task 1: Autonomous SLAM Mapping

**Stack:** `slam_toolbox` + `explore_lite` + `Nav2`

The robot explores using frontier-based exploration (`explore_lite` continuously sends Nav2 goals to the nearest unmapped frontier). The laser footprint filter removes the robot body from the scan (inscribed radius 0.28 m) to prevent phantom obstacles.

Map completion is detected by `frontier_map_saver` without a timer:

```python
# frontier_map_saver.py - convergence logic (simplified)
delta = known_cells - self._baseline_known
if abs(delta) >= self.change_threshold:      # 50 cells
    self._last_change_time = now
    self._baseline_known   = known_cells     # reset baseline, not prev message

# Comparison is against the baseline, not the previous message.
# This accumulates slow sub-threshold growth and avoids a false "done"
# while the map is still filling in below threshold per step.
stable_t = (now - self._last_change_time).nanoseconds * 1e-9
if stable_t >= self.stable_seconds and elapsed >= self.min_explore_s and self._grew:
    self._save_map()    # calls /map_saver/save_map service, no direct file I/O
```

**Key design choices:**
- Map is saved via the `nav2_msgs/SaveMap` service rather than direct file I/O, which keeps it ROS-native.
- Completion requires minimum explore time (20 s), actual map growth (not just stability at start), and 35 s of stability.
- Launch is fully event-driven: `OnExecutionComplete` and `OnProcessExit` handlers chain every subsystem. No `sleep()` timers in the launch file.

---

## Task 2: Re-localization and ArUco Discovery

### Autonomous Localization

The robot starts at an unknown pose. `autonomous_localization_initial` drives AMCL to convergence:

```python
# autonomous_localization_initial.py - convergence loop (simplified)
def localization(self):
    while True:
        self.micro_explore()        # random small moves to gather laser information
        self._clear_costmaps()      # wipe phantom obstacles each iteration

        if np.max(self.covariance_msg.pose.covariance) < self.covariance_threshold:
            self.localization_done_pub.publish(Bool(data=True))
            break

def micro_explore(self, steps=5, max_dist=0.4, max_angle=0.6):
    for _ in range(steps):
        vel = Twist()
        if self.min_dist < self.OBSTACLE_DIST:   # laser says obstacle ahead
            vel.linear.x  = -0.08               # back off
            vel.angular.z =  max_angle
        else:
            vel.linear.x  = random.uniform(0.05, max_dist)
            vel.angular.z = random.uniform(-max_angle, max_angle)
        self.publisher.publish(vel)
        time.sleep(0.3)
```

Convergence criterion: `max(covariance_matrix) < 0.1`. Costmaps are cleared each iteration so AMCL accumulated information is not obscured by phantom obstacles from the random motion.

### Viewpoint Planning

`viewpoint_planner` computes safe camera viewpoints from the static map geometry:

```python
# viewpoint_planner.py - standoff pose computation (simplified)
# For each wall segment longer than MIN_SEGMENT_CELLS:
#   sample boundary points every VIEWPOINT_SPACING metres
#   compute inward wall normal from neighbourhood
#   place viewpoint at STANDOFF_DISTANCE + PULLBACK_DISTANCE along normal
#   filter: costmap cost < 90, min distance from existing viewpoints

normal_angle = atan2(ny, nx)
vx = bx + (STANDOFF_DISTANCE + PULLBACK_DISTANCE) * nx
vy = by + (STANDOFF_DISTANCE + PULLBACK_DISTANCE) * ny
yaw = normal_angle + math.pi    # face the wall
```

The action server is only created *after* computation finishes, so `mission_executor` cannot receive a result before it is ready: no polling, no race condition.

### ArUco Discovery State Machine

`mission_executor` orders viewpoints by greedy nearest-neighbour from the robot's post-localization pose, then navigates through them:

```python
# mission_executor.py - ArUco detection and pause/resume (simplified)
def _pick_aruco_cb(self, msg):
    self.pick_count -= 1
    if not self.aruco_pick_seen:
        self.aruco_pick_seen = True
        self._align_head_to_marker('aruco_pick_frame')
        self._stop_robot()          # cancel Nav2 goal, save interrupted pose

    if self.pick_count <= 0:        # 20 stable frames accumulated
        self._get_table_frames_from_aruco(msg, ...)
        self.aruco_pick_saved = True
        self._resume_robot()        # re-send the interrupted Nav2 goal

def _stop_robot(self):
    self._paused_for_aruco = True
    self._interrupted_goal = self._last_sent_goal   # save for resume
    self.current_goal_handle.cancel_goal_async()
    self.cmd_vel_pub.publish(Twist())               # zero velocity
```

The ArUco watchdog aborts a measurement if the marker disappears for more than 6 seconds and resumes exploration, preventing a deadlock.

---

## Task 3: Pick and Place

### Coarse-to-Fine Table Approach

The YAML pose from Task 2 navigates the robot to roughly in front of the table. On arrival, ArUco is re-detected live to recompute the exact approach pose at centimetre precision, compensating for AMCL and navigation drift.

### Cube Handling

Each cube is one entry in a dictionary keyed by ArUco ID. Adding more cubes requires only a new dictionary entry; the state machine, detection pipeline, and manipulation logic are all unchanged:

```python
# pick_and_place.py - cube registry
@dataclass
class CubeData:
    id: int
    name: str
    seen: bool  = False
    placed: bool = False
    pick_center:   Frame = None    # detected at runtime from ArUco
    pick_approach: Frame = None    # 25 cm above cube
    pick_target:   Frame = None    # 6 cm above cube, grasp point
    T_table_cube:  Frame = None    # relative TF reused on place table

self.cubes = {
    63:  CubeData(id=63,  name='aruco_cube_exam_id63'),
    582: CubeData(id=582, name='aruco_cube_exam_id582'),
}
```

The place pose is never hardcoded: the offset from the pick table ArUco to the cube is measured at pick time, then re-applied on the place table so the robot works regardless of cube or table position.

### MoveIt 2 Manipulation

Collision objects are modified at runtime: the cube is excluded from the collision scene during planning, then re-included after the gripper closes. A `link_attacher` plugin rigidly attaches the cube after gripper close, which is the standard workaround for small-object contact instability in Gazebo.

---

## Launch

```bash
# x_pose / y_pose set only the robot's initial spawn pose in Gazebo.
# All task-relevant poses (tables, cubes, grasps) are discovered at runtime.

# Task 1: autonomous mapping
ros2 launch tiago_exam task_1.launch.py world_name:=group11 moveit:=true x_pose:=-1.0 y_pose:=-1.0

# Task 2: re-localization + table discovery (requires Task 1 map)
ros2 launch tiago_exam task_2.launch.py world_name:=group11 moveit:=true x_pose:=1.0 y_pose:=-3.0

# Task 3: pick and place (requires Task 2 frames)
ros2 launch tiago_exam task_3.launch.py world_name:=group11 moveit:=true x_pose:=4.0 y_pose:=-1.0

# Task 3 continuing from Task 2 without restarting Gazebo
ros2 launch tiago_exam task_3.launch.py world_name:=group11 moveit:=true task_2:=True
```

---

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stable_seconds` | 35 s | Map stability window before saving |
| `min_explore_seconds` | 20 s | Minimum exploration time before save is allowed |
| `change_threshold` | 50 cells | Minimum cell change to reset stability timer |
| `covariance_threshold` | 0.1 | Max AMCL covariance to declare localization done |
| `viewpoint_spacing` | 0.8 m | Arc-length between sampled boundary points |
| `min_global_spacing` | 0.8 m | Minimum Euclidean gap between final viewpoints |
| `inflation_radius` | 0.6 m | Must match `global_costmap` inflation radius |
| `approach_distance` | 25 cm | Pre-grasp standoff above cube |
| `target_distance` | 6 cm | Final grasp point above cube top face |

---

## Dependencies

- ROS 2 Humble
- Nav2 (AMCL, costmaps, planner, controller)
- MoveIt 2
- `slam_toolbox`
- `explore_lite`
- `aruco_ros`
- `pymoveit2`
- `gazebo_ros_link_attacher`
- `scipy`, `numpy`, `PyKDL`

---

## Source Code

This is a public showcase. The complete, runnable source code is maintained in a private repository and is **available on request**: open an issue or reach out to any of the authors below.

## Authors

- [Broccoli Thomas](https://github.com/broccolithomas01)
- [Tavini Gabrielmario](https://github.com/gabrielmario-tavini)
- [Ziosi Nicolò](https://github.com/zionico)
