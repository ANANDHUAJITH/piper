# 🐛 Known Issues & Workarounds

## Issue 1 — `draw_square()` and `safe_move_linear()` Do Not Work

### Symptom
Running the `SQ` command prints:
```
❌ No linear move in SDK version!
```
The arm does not move.

### Root Cause
`safe_move_linear()` attempts to find a Cartesian linear-move method on the SDK object:

```python
def safe_move_linear(piper, x, y, z, rx, ry, rz, speed):
    if hasattr(piper, "LinearMove"):
        return piper.LinearMove(...)
    if hasattr(piper, "MovePose"):
        return piper.MovePose(...)
    print("❌ No linear move in SDK version!")
    return None
```

The installed version of `piper_sdk` (`C_PiperInterface_V2`) does **not** expose either `LinearMove` or `MovePose` as public methods. Cartesian-space motion is not currently accessible through this SDK version.

### Workaround
To trace a square or any shape, define the corner positions as joint-space waypoints in Mode 3 and execute with `XD` (Direct) or `B` (B-spline).

**Steps:**
1. Manually jog the arm to each corner of the desired shape using Mode 1 joint commands
2. At each corner, press `W` to record the waypoint
3. In Mode 3, use `XD` to move directly through all corners

### Future Fix
If a newer SDK version adds `EndPoseCtrl`, `SetEndPose`, or equivalent Cartesian control, update `safe_move_linear()` to call that method.

---

## Issue 2 — Speed Setting Does Not Persist Between Modes

### Symptom
Speed set with `V5` in Mode 1 appears to reset when switching to Mode 2.

### Root Cause
The `SpeedController` object is shared across modes (passed by reference), so the `current_speed` value is preserved. However, the SDK's internal state may reset after certain calls. The `MotionCtrl_2` command needs to be re-sent.

### Workaround
Re-set speed at the beginning of each mode session with `V<n>`.

---

## Issue 3 — `GetArmEndPoseMsgs()` Returns Zeros in Simulation

### Symptom
`P` command prints all zeros for X, Y, Z in RViz/MoveIt simulation.

### Root Cause
The MoveIt simulation bridge does not populate end-effector pose messages the same way the hardware does. Joint states are forwarded correctly but the pose message is not computed from FK.

### Workaround
Use `P` to verify position on hardware only. In simulation, use `L` to check joint angles instead.

---

## Issue 4 — Emergency Stop Does Not Physically Stop the Hardware Mid-Motion

### Symptom
Pressing `E` during a trajectory sets the software flag, but the robot may complete its current joint command before stopping.

### Root Cause
Commands are sent to the robot over CAN bus and executed by the motor controllers. Once a `JointCtrl` packet is sent, the robot will complete that motion step. The software `e_stop` flag is checked **between** trajectory points, not mid-motion.

### Workaround
Use shorter `time_per_point` values (1–2s) so the loop checks the flag more frequently. For a true hardware e-stop, use the physical emergency stop button on the robot if available.

---

## Issue 5 — Quintic Trajectory `Q` Command Conflicts with Quit `Q` in Mode 3

### Symptom
In Mode 3 (Trajectory Planning), pressing `Q` launches the Quintic trajectory planner instead of quitting.

### Root Cause
The `Q` key is overloaded: in Modes 1 and 2 it quits, but in Mode 3 it was reassigned to Quintic trajectory. This is intentional but may confuse users.

### Workaround
In Mode 3, use `M` to return to the mode menu and `X` to exit trajectory mode. Use `Q` only to run the Quintic trajectory planner.
