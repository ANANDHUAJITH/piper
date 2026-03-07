# 📚 PIPER Arm — Complete Function Reference

> Every class and function in `piper_arm_hardware.py`, explained in plain language with examples.

---

## Table of Contents

1. [RobotLogger](#1-robotlogger)
2. [BSplineTrajectory](#2-bsplinetrajectory)
3. [WaypointManager](#3-waypointmanager)
4. [EmergencyStop](#4-emergencystop)
5. [SpeedController](#5-speedcontroller)
6. [Utility Functions](#6-utility-functions)
7. [Operating Modes](#7-operating-modes)
8. [Main Program](#8-main-program)
9. [Known Non-Working Functions](#9-known-non-working-functions)

---

## 1. RobotLogger

**File location:** Top of `piper_arm_hardware.py`

### Purpose
Records every robot command and its resulting position to a CSV file. The logger accumulates data in memory and only writes to disk when you press `S` (save) or quit. This avoids slow file I/O during motion.

---

### `__init__(self, mode_name)`

```python
logger = RobotLogger("Mode1_Interactive")
```

**What it does:**
- Asks you to choose a filename or accept the auto-generated default
- Creates the save folder `~/Desktop/PIPER_CSV/` if it doesn't exist
- Sets up an empty in-memory buffer (`self.buffer = []`)

**Parameters:**
- `mode_name` — a label that becomes part of the default filename (e.g. `"Mode3_Trajectory"`)

**Auto-generated filename example:**
```
robot_log_Mode3_Trajectory_20250210_143022.csv
```

---

### `log(self, command, joints, gripper, pose)`

```python
logger.log("AP10", [10000, 0, 0, 0, 0, 0], 5000, pose_msg)
```

**What it does:**
Appends one row to the in-memory buffer. Does **not** write to disk yet.

**Parameters:**
- `command` — the string command that caused this state (e.g. `"AP10"`, `"BSPLINE_3"`)
- `joints` — list of 6 joint angles in **millidegrees** (the raw SDK unit)
- `gripper` — gripper angle in **millidegrees** (raw SDK unit)
- `pose` — the full pose message from `piper.GetArmEndPoseMsgs()`, or `None`

**Unit conversion done internally:**
- Joints: millidegrees → degrees (`j / 1000`)
- Gripper: millideg → mm (`g / 1000`)
- Pose X/Y/Z/RX/RY/RZ: raw units → mm or degrees (`/ 1000`)

---

### `save(self)`

```python
logger.save()
```

**What it does:**
Writes the entire in-memory buffer to disk as a CSV file with a header row.

**CSV columns written:**
```
Timestamp, Command,
Joint1_deg, Joint2_deg, Joint3_deg, Joint4_deg, Joint5_deg, Joint6_deg,
Gripper_mm,
EndEff_X_mm, EndEff_Y_mm, EndEff_Z_mm,
EndEff_RX_deg, EndEff_RY_deg, EndEff_RZ_deg
```

**Returns:** `True` on success, `False` on failure.

---

### `close(self)`

```python
logger.close()
```

**What it does:**
Called automatically when switching modes or quitting. If there is unsaved data in the buffer, it calls `save()` automatically so you don't lose data.

---

## 2. BSplineTrajectory

**Purpose:** Generates smooth interpolated paths between waypoints using two different mathematical methods.

> ⚠️ This class does **not** move the robot — it only generates a list of intermediate positions. The trajectory execution loop in Mode 3 does the actual movement.

---

### `cubic_bspline_basis(t)` — static method

```python
b0, b1, b2, b3 = BSplineTrajectory.cubic_bspline_basis(0.5)
```

**What it does:**
Computes the four cubic B-spline blending functions at parameter `t`.

**Parameter:**
- `t` — a value between 0.0 and 1.0 representing position along a curve segment

**Returns:** Four floats `(b0, b1, b2, b3)` that sum to 1.0. They are used as weights to blend four nearby control points into one smooth point.

**Math background:**
The cubic B-spline basis ensures the curve passes smoothly near (but not necessarily through) each control point. The curve is C² continuous — meaning both velocity and acceleration are smooth at every point.

---

### `generate_trajectory(waypoints, num_points=50)` — static method

```python
trajectory = BSplineTrajectory.generate_trajectory(my_waypoints, num_points=10)
```

**What it does:**
Takes your list of waypoints and produces a longer list of smooth intermediate positions using cubic B-spline interpolation.

**Parameters:**
- `waypoints` — list of positions, each with 7 values: `[J1, J2, J3, J4, J5, J6, Gripper_mm]` (all in degrees/mm)
- `num_points` — total number of interpolated points to generate (default 50; recommended 5–10 for hardware)

**Returns:**
A list of interpolated positions in the same format as the input waypoints.

**Important behaviour:**
- Requires at least 2 waypoints
- If fewer than 4 waypoints are given, the first and last points are duplicated internally to ensure enough control points for the cubic formula
- The curve does **not** pass exactly through the waypoints — it passes smoothly near them

**Recommended settings for hardware:**
```
num_points = 5–10
time_per_point = 3–5 seconds
```

---

### `generate_quintic_trajectory(waypoints, num_points=50)` — static method

```python
trajectory = BSplineTrajectory.generate_quintic_trajectory(my_waypoints, num_points=10)
```

**What it does:**
Produces a smooth trajectory using a **quintic (5th-order) polynomial** between each pair of consecutive waypoints.

**The quintic formula used:**
```
s(t) = 10t³ - 15t⁴ + 6t⁵
```
This is chosen specifically because at `t=0`: s=0, ṡ=0, s̈=0, and at `t=1`: s=1, ṡ=0, s̈=0. In plain terms: the robot **starts and ends each segment with zero velocity and zero acceleration** — giving very smooth, jerk-free motion.

**Key difference from B-spline:**
- B-spline: globally smooth, does not guarantee zero velocity at waypoints
- Quintic: zero velocity at every waypoint → more controlled but slightly slower feel

**Parameters and return value:** Same as `generate_trajectory`.

---

## 3. WaypointManager

**Purpose:** Stores, lists, and manages waypoints during a session. Waypoints are held in memory only — they are lost when the program exits unless saved to a `.txt` file at shutdown.

---

### `__init__(self)`

```python
waypoint_mgr = WaypointManager()
```

Creates an empty waypoints list. Each waypoint is stored as `[J1, J2, J3, J4, J5, J6, Gripper_mm]` in degrees/mm.

---

### `add_current(self, joints, gripper)`

```python
wp_num = waypoint_mgr.add_current(current_joints, current_gripper)
```

**What it does:**
Reads the robot's live joint and gripper values, converts them from raw millidegrees to degrees/mm, and appends them as a new waypoint.

**Parameters:**
- `joints` — list of 6 values in millidegrees (from `safe_read_joints()`)
- `gripper` — single value in millidegrees (from `safe_read_gripper()`)

**Returns:** The waypoint number (1-indexed, for display purposes).

---

### `add_manual(self, values)`

```python
waypoint_mgr.add_manual([10.0, 20.0, 30.0, 0.0, 0.0, 0.0, 50.0])
```

**What it does:**
Adds a waypoint with manually specified values (used by the `W J1,J2,...,Gripper` syntax in Mode 3).

**Parameters:**
- `values` — a list of exactly 7 floats: `[J1, J2, J3, J4, J5, J6, Gripper_mm]`

**Returns:** `True` if successful, `False` if the wrong number of values was given.

---

### `list_all(self)`

```python
waypoint_mgr.list_all()
```

Prints all stored waypoints to the terminal in a readable format. Used by the `L` command.

---

### `clear(self)`

```python
waypoint_mgr.clear()
```

Removes all stored waypoints. Used by the `CW` command.

---

### `get_all(self)`

```python
waypoints = waypoint_mgr.get_all()
```

Returns a copy of the full waypoints list. Used internally by trajectory execution.

---

### `delete(self, index)`

```python
removed = waypoint_mgr.delete(2)   # Delete 3rd waypoint (0-indexed)
```

Removes a single waypoint by its zero-based index.

**Returns:** The deleted waypoint (a list of 7 values) if successful, or `None` if the index was out of range.

---

## 4. EmergencyStop

**Purpose:** A simple flag object that any part of the program can check before moving the robot.

---

### `activate(self)`

```python
e_stop.activate()
```

Sets the stop flag to `True` and prints a warning. Called by the `E` command.

---

### `deactivate(self)`

```python
e_stop.deactivate()
```

Clears the stop flag. Called by the `R` command.

---

### `check(self)`

```python
if e_stop.check():
    break  # Halt trajectory
```

**Returns:** `True` if emergency stop is active, `False` otherwise.

Used inside every trajectory loop and at the top of `execute_command()` to prevent any motion while the stop is active.

---

## 5. SpeedController

**Purpose:** Wraps the SDK's `MotionCtrl_2` call to provide a simple numbered speed scale (1–10).

---

### `__init__(self, piper, default_speed=3)`

```python
speed_ctrl = SpeedController(piper, default_speed=8)
```

Stores the piper interface and sets the initial speed value. Does **not** send the command to the robot yet — `set_speed()` must be called separately.

---

### `set_speed(self, speed)`

```python
speed_ctrl.set_speed(5)
```

**What it does:**
Validates the speed (must be 1–10), then calls `piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)` to apply it.

**Parameters:**
- `speed` — integer 1–10

**Speed guide:**
| Speed | Description |
|-------|-------------|
| 1–3 | Slow — use for first hardware tests |
| 4–6 | Medium |
| 7–8 | Fast (default: 8) |
| 9–10 | Maximum — use with caution |

**Returns:** `True` on success, `False` if out of range.

---

### `get_speed(self)`

```python
current = speed_ctrl.get_speed()
```

Returns the current speed setting as an integer.

---

### `print_speed_info(self)`

```python
speed_ctrl.print_speed_info()
```

Prints the current speed and a human-readable scale to the terminal. Called by the `V` command with no number.

---

## 6. Utility Functions

### `safe_read_joints(piper)`

```python
joints = safe_read_joints(piper)
# Returns: [j1, j2, j3, j4, j5, j6] in millidegrees, or None
```

**What it does:**
Calls `piper.GetArmJointMsgs()` and extracts the six joint values. Returns `None` if the call fails, protecting the rest of the code from crashes.

---

### `safe_read_gripper(piper)`

```python
gripper = safe_read_gripper(piper)
# Returns: gripper angle in millidegrees, or None
```

**What it does:**
Calls `piper.GetArmGripperMsgs()` and extracts the gripper angle. Returns `None` on failure.

---

### `print_position(piper)`

```python
print_position(piper)
```

**What it does:**
Reads the end-effector pose via `piper.GetArmEndPoseMsgs()` and prints X, Y, Z (in mm) and RX, RY, RZ (in degrees) to the terminal. Triggered by the `P` command.

---

### `safe_move_linear(piper, x, y, z, rx, ry, rz, speed)`

```python
safe_move_linear(piper, 300000, 0, 200000, 0, 0, 0, 10)
```

> ❌ **This function does not work with the current SDK version.**

**Intent:** Attempt to call a Cartesian-space linear move using whichever method exists on the SDK object — first tries `LinearMove`, then `MovePose`.

**Why it fails:** Neither `LinearMove` nor `MovePose` is available in the installed version of `piper_sdk`. See `docs/KNOWN_ISSUES.md`.

---

### `draw_square(piper, side=50)`

```python
draw_square(piper, side=50)
```

> ❌ **This function does not work with the current SDK version.**

**Intent:** Read the current end-effector position and move in a 50mm square in the YZ plane using four linear moves.

**Why it fails:** Depends on `safe_move_linear()`, which is broken. See `docs/KNOWN_ISSUES.md`.

---

### `execute_command(cmd, piper, joints, gripper, logger, waypoint_mgr, e_stop, speed_ctrl)`

```python
joints, gripper, should_continue = execute_command(
    "AP10", piper, joints, gripper, logger, waypoint_mgr, e_stop, speed_ctrl
)
```

**What it does:**
The central command dispatcher. Takes a single command string, parses it, and calls the appropriate robot action. This function is used by **all three modes**.

**Returns:** A tuple `(joints, gripper, should_continue)` where `should_continue` is `False` only when the `Q` (quit) command is received.

**Commands handled:**
| Pattern | Action |
|---------|--------|
| `E` | Activate emergency stop |
| `R` | Deactivate emergency stop |
| `V`, `V1`–`V10` | Speed control |
| `S` | Save CSV |
| `P` or `C` | Print position |
| `W` | Add current waypoint |
| `L` | List waypoints |
| `CW` | Clear waypoints |
| `Q` | Signal quit |
| `Z` | Zero/home all joints |
| `SQ` | Draw square (broken) |
| `AP10`, `BN05`, etc. | Move joint by degrees |
| `G10` | Open gripper by mm |
| `F10` | Close gripper by mm |

---

## 7. Operating Modes

### `mode_command_by_command(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl)`

Runs an interactive loop that reads one command at a time from `input()` and passes it to `execute_command()`. Typing `M` returns to the mode menu; `Q` quits the program.

Creates its own `RobotLogger` labelled `"Mode1_Interactive"`.

---

### `mode_sequence(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl)`

Reads a full line of space-separated commands, splits them, and executes each one in order with a 0.1s pause between them. Stops the sequence (but not the program) if an emergency stop is triggered mid-sequence.

Creates its own `RobotLogger` labelled `"Mode2_Sequence"`.

---

### `mode_trajectory_planning(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl)`

The most complex mode. Handles all waypoint management commands, trajectory generation, and execution in one large command loop.

Key sub-behaviours:
- `B` → generates B-spline trajectory, asks for num_points and time_per_point, then runs the loop
- `Q` → same as B but uses quintic polynomial
- `XD` → skips interpolation and moves directly through each stored waypoint

Creates its own `RobotLogger` labelled `"Mode3_Trajectory"`.

---

### `show_mode_menu()`

Prints the mode selection screen and returns the user's input string. Called at the top of the main loop.

---

## 8. Main Program

### `main()`

Entry point. Does the following in order:

1. Creates `WaypointManager` and `EmergencyStop` instances (shared across all modes)
2. Creates the `C_PiperInterface_V2` object with software joint and gripper limits enabled
3. Calls `piper.ConnectPort()` and loops on `piper.EnablePiper()` until successful
4. Creates `SpeedController` and applies default speed of 8/10
5. Calls `piper.CrashProtectionConfig(1,1,1,1,1,1)` for maximum sensitivity
6. Reads initial joint and gripper state
7. Enters the main mode-selection loop
8. On exit/interrupt, offers to save waypoints to a `.txt` file

**SDK initialisation flags:**
```python
piper = C_PiperInterface_V2(
    "can0",
    start_sdk_joint_limit=True,     # Enforce joint angle limits in software
    start_sdk_gripper_limit=True    # Enforce gripper range limits in software
)
```

---

## 9. Known Non-Working Functions

See `docs/KNOWN_ISSUES.md` for full details. Summary:

| Function | Issue |
|----------|-------|
| `safe_move_linear()` | `LinearMove` and `MovePose` not in installed SDK |
| `draw_square()` | Depends on `safe_move_linear()` |
| `SQ` command | Calls `draw_square()` |

**Workaround for square/shape drawing:**
Use joint-space waypoints in Mode 3 with `XD` (Direct) or `B` (B-spline) execution. This achieves similar results through joint-space interpolation rather than Cartesian-space linear moves.
