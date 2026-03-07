# PiPER Robotic Arm — Student Experiment Manual
### TKM College of Engineering, Kollam

---

## ⚠️ SAFETY RULES

- Clear workspace — **1 metre radius** around robot at all times
- Test in **simulation BEFORE hardware**
- Press **E** for emergency stop if anything unexpected happens
- Never put hands near the robot during motion
- Hardware sessions require **instructor sign-off**

---

## PART 1: SIMULATION (RViz / MoveIt)

### STEP 1 — Open Terminal

Press the **Super** key (Windows key), type **Terminator**, press Enter.

**Useful terminal shortcuts:**
```
Ctrl+Shift+C   →  Copy
Ctrl+Shift+V   →  Paste
Ctrl+C         →  Stop running program
Ctrl+Shift+E   →  Split terminal (open new pane)
```

---

### STEP 2 — Source Workspace (Pane 1)

```bash
cd ~/piper_ws
source devel/setup.bash
```

---

### STEP 3 — Launch RViz (Pane 1)

```bash
cd ~/piper_ws/src/piper_with_gripper_moveit/launch
roslaunch demo.launch
```

Wait for the RViz window showing the 3D robot model to appear.

⚠️ **Keep this terminal running throughout the session.**

---

### STEP 4 — Open a New Pane

Press `Ctrl+Shift+E` to split the terminal vertically.

---

### STEP 5 — Navigate to Controller (Pane 2)

```bash
cd ~/Desktop/PIPER_STUDENTS
```

---

### STEP 6 — Make Script Executable

```bash
chmod +x piper_arm_hardware.py
```

---

### STEP 7 — Run the Controller

```bash
python3 piper_arm_hardware.py
```

You will see the mode selection menu:
```
Select mode (1-3) or Q to quit:
```

---

## THE THREE MODES

### MODE 1 — Command-by-Command
- Type **one command** at a time
- Robot executes immediately
- Best for learning individual joints and testing

**Example session:**
```
CMD > AP10     # Joint 1 moves +10°
CMD > BP20     # Joint 2 moves +20°
CMD > W        # Save this position as a waypoint
```

---

### MODE 2 — Sequence Mode
- Type **multiple commands** on one line, separated by spaces
- Robot executes all commands in order
- Best for repeated pick-and-place patterns

**Example:**
```
SEQUENCE > AP10 BP20 G05 W F10 W P
```

---

### MODE 3 — Trajectory Planning
- Build up a list of waypoints
- Execute a smooth interpolated path between them
- Supports B-spline and Quintic polynomial interpolation
- Best for smooth motion research and comparison

**Example:**
```
TRAJ > W            # Record waypoint 1
TRAJ > AP30 BP20    # Move arm
TRAJ > W            # Record waypoint 2
TRAJ > L            # List waypoints
TRAJ > B            # Execute B-spline trajectory
```

---

## COMMAND REFERENCE

### Joint Movement

Pattern: `<Joint><Direction><Degrees>`

| Joint | Letter |
|-------|--------|
| Joint 1 (base rotation) | A |
| Joint 2 (shoulder) | B |
| Joint 3 (elbow) | C |
| Joint 4 (forearm roll) | D |
| Joint 5 (wrist pitch) | E |
| Joint 6 (wrist roll) | F |

Direction: `P` = positive, `N` = negative

Examples:
```
AP10   →  Joint 1, +10°
AN10   →  Joint 1, -10°
BP20   →  Joint 2, +20°
BN05   →  Joint 2, -5°
CP15   →  Joint 3, +15°
```

---

### Gripper

| Command | Action |
|---------|--------|
| `G10` | Open gripper +10mm |
| `F10` | Close gripper -10mm |

Gripper range: 0mm (fully closed) to 80mm (fully open).

---

### Waypoints

| Command | Action |
|---------|--------|
| `W` | Save current position as waypoint |
| `L` | List all saved waypoints |
| `CW` | Clear all waypoints |
| `V 3` | View waypoint number 3 |
| `D 2` | Delete waypoint number 2 |

In Mode 3 you can also add waypoints manually:
```
TRAJ > W 10,20,30,0,0,0,50
```
Format: `J1,J2,J3,J4,J5,J6,Gripper_mm`

---

### Trajectory Execution (Mode 3 only)

| Command | Method | Description |
|---------|--------|-------------|
| `B` | B-spline | Smooth curves through near all waypoints |
| `Q` | Quintic polynomial | Zero velocity at each waypoint |
| `XD` | Direct point-to-point | No interpolation, exact waypoints only |

---

### Information & Control

| Command | Action |
|---------|--------|
| `P` | Print end-effector XYZ position |
| `V` | Show current speed |
| `V5` | Set speed to 5 out of 10 |
| `E` | Emergency stop (halts all motion) |
| `R` | Resume after emergency stop |
| `Z` | Reset arm to home (all joints = 0°) |
| `S` | Save CSV log to file |
| `M` | Return to mode selection menu |
| `Q` | Quit program |

---

## EXPERIMENT LEARNING SEQUENCE

### PHASE 1 — Mode 1 Practice (15 minutes)

**Objective:** Get familiar with individual joint control and basic commands.

```
Select mode (1-3) or Q to quit: 1
```

Test each joint one at a time. Watch each joint move in the RViz window:
```
CMD > AP10      # Joint 1 +10°
CMD > AN10      # Joint 1 back -10°
CMD > BP20      # Joint 2 +20°
CMD > BN20      # Joint 2 back -20°
CMD > CP15      # Joint 3 +15°
CMD > CN15
CMD > DP05
CMD > DN05
CMD > EP30
CMD > EN30
CMD > FP10
CMD > FN10
```

Test the gripper:
```
CMD > G10       # Open 10mm
CMD > F10       # Close 10mm
```

Check the end-effector position:
```
CMD > P         # Prints X, Y, Z and orientation
```

Save some waypoints:
```
CMD > W         # Save current position
CMD > AP20 BP15
CMD > W         # Save new position
CMD > L         # List waypoints — you should see 2
```

Test emergency stop:
```
CMD > E         # Stop
CMD > R         # Resume
```

Return to menu:
```
CMD > M
```

---

### PHASE 2 — Mode 2 Practice (15 minutes)

**Objective:** Programme repeatable multi-step sequences.

```
Select mode (1-3) or Q to quit: 2
```

Simple sequence:
```
SEQUENCE > AP10 BP10 W P
```

A pick-and-place style pattern:
```
SEQUENCE > AP20 BP15 W
SEQUENCE > F10 W
SEQUENCE > BN10 W
SEQUENCE > AN20 W
SEQUENCE > G10 W
SEQUENCE > L
```

Return to menu:
```
SEQUENCE > M
```

---

### PHASE 3 — Mode 3 Trajectory Comparison (30 minutes)

**Objective:** Compare B-spline and Quintic trajectory smoothness.

```
Select mode (1-3) or Q to quit: 3
```

**Part A — B-spline Trajectory**

Create 20–30 waypoints (small incremental steps give a smooth comparison):
```
TRAJ > W                    # Start position — Waypoint 1
TRAJ > AP05 BP05 W          # Waypoint 2
TRAJ > AP05 BP05 W          # Waypoint 3
TRAJ > AP05 BP05 W          # Waypoint 4
TRAJ > AP05 BP05 W          # Waypoint 5
TRAJ > AP05 BN05 W          # Waypoint 6 (change direction)
TRAJ > AP05 BN05 W          # Waypoint 7
TRAJ > AP05 BN05 W          # Waypoint 8
... (continue until you have 20–30 waypoints)
```

List your waypoints:
```
TRAJ > L
```

Execute B-spline:
```
TRAJ > B
Number of interpolation points: 10
Seconds per point: 3
Execute B-spline trajectory? (y/n): y
```

Save the CSV — **note the filename**:
```
TRAJ > S
```

---

**Part B — Quintic Trajectory (Same Waypoints)**

Clear waypoints and recreate the **same sequence**:
```
TRAJ > CW
(Recreate the same 20–30 waypoints from Part A)
```

Execute Quintic:
```
TRAJ > Q
Number of interpolation points: 10
Seconds per point: 3
Execute Quintic trajectory? (y/n): y
```

Save the CSV — **note this filename too**:
```
TRAJ > S
```

---

## PLOTTING AND ANALYSIS

### Find Your CSV Files

```bash
cd ~/Desktop/PIPER_CSV
ls -lh
```

You should see files like:
```
robot_log_Mode3_Trajectory_20250210_143022.csv
robot_log_Mode3_Trajectory_20250210_145510.csv
```

---

### Plot a Single File

```bash
cd ~/Desktop/PIPER_STUDENTS
python3 plot_trajectory.py ~/Desktop/PIPER_CSV/robot_log_Mode3_Trajectory_20250210_143022.csv
```

This creates three graphs:
- `*_joint_space.png` — All 6 joint angles over time
- `*_cartesian_space.png` — End-effector X, Y, Z path
- `*_smoothness.png` — Joint acceleration (lower = smoother)

---

### Compare B-spline vs Quintic

```bash
python3 plot_trajectory.py \
  ~/Desktop/PIPER_CSV/robot_log_BSPLINE.csv \
  ~/Desktop/PIPER_CSV/robot_log_QUINTIC.csv
```

Creates a side-by-side comparison graph.

---

### View Graphs

```bash
eog *.png
```

Or copy to Desktop:
```bash
cp *.png ~/Desktop/
```

---

## UNDERSTANDING THE GRAPHS

**Joint Space Graph**
Shows each joint's angle (in degrees) over time. Smooth continuous curves indicate good trajectory execution. Sudden jumps or steps indicate the robot may not have completed each motion before the next command arrived.

**Cartesian Space Graph**
Shows the end-effector's position in 3D space (X, Y, Z in mm). This is different from joint space — the same joint motion can create very different Cartesian paths depending on the arm's configuration.

**Smoothness / Acceleration Graph**
Shows the rate of change of velocity for each joint. Lower values mean less jerk. Compare this between B-spline and Quintic to see which produces smoother motion. Quintic typically shows lower acceleration peaks near waypoints because it enforces zero velocity at each stop.

**Comparison Graph**
Solid lines = Method 1 (B-spline). Dashed lines = Method 2 (Quintic). Look for differences in:
- Path smoothness
- Overshoot near waypoints
- Acceleration peaks

---

## PART 2: HARDWARE (Instructor Approval Required)

### ⚠️ Pre-Hardware Checklist

- [ ] All simulation exercises completed
- [ ] Instructor has signed off
- [ ] Workspace cleared (1m radius)
- [ ] Emergency stop location known
- [ ] All team members alert and watching

---

### STEP 1 — Source Workspace

```bash
cd ~/piper_ws
source devel/setup.bash
```

---

### STEP 2 — Bring Up CAN Bus

```bash
sudo ip link set can0 up type can bitrate 1000000
```

Enter your password when prompted.

Check status:
```bash
ip -details -statistics link show can0
```

Look for: `state ERROR-ACTIVE` (this means CAN is ready).

---

### STEP 3 — Verify Motor Communication

```bash
candump can0
```

You should see scrolling CAN frames. If nothing appears, check that the robot is powered and cables are connected.

Press `Ctrl+C` to stop.

---

### STEP 4 — Run the Hardware Controller

```bash
cd ~/Desktop/PIPER_STUDENTS
python3 piper_arm_hardware.py
```

---

### STEP 5 — Safety Tests First

Set minimum speed:
```
Select mode: 1
CMD > V1
```

Test emergency stop:
```
CMD > E
CMD > R
```

Test the smallest possible movement:
```
CMD > AP01
CMD > AN01
```

⚠️ Watch closely. If any unexpected motion occurs, press `E` immediately.

---

### STEP 6 — Run Your Verified Sequence

Use the same sequences you tested in simulation:
```
Select mode: 2
SEQUENCE > AP10 BP10 W P
```

Or run Mode 3 with the same waypoints from simulation. **Use 5 seconds per point on hardware.**

---

### STEP 7 — Save Data

```
CMD > S
```

---

### STEP 8 — Safe Shutdown

```
CMD > Z          # Return to home position
CMD > Q          # Quit
```

Bring down CAN bus:
```bash
sudo ip link set can0 down
```

---

## COMPARING SIMULATION VS HARDWARE

```bash
python3 plot_trajectory.py \
  ~/Desktop/PIPER_CSV/robot_log_SIMULATION.csv \
  ~/Desktop/PIPER_CSV/robot_log_HARDWARE.csv
```

Look for differences in:
- Actual vs commanded joint positions
- Path accuracy
- Motion smoothness

---

## TROUBLESHOOTING

| Problem | Solution |
|---------|----------|
| RViz doesn't open | Run `source devel/setup.bash` again |
| Robot doesn't move in RViz | Restart `python3 piper_arm_hardware.py` |
| CAN state shows DOWN | Run `sudo ip link set can0 up type can bitrate 1000000` |
| `candump` shows nothing | Check robot power and CAN cable connection |
| Plot script gives error | Run `pip3 install pandas matplotlib` |
| All positions read as zero | Known issue in simulation — use hardware for position data |

---

## KEY CONCEPTS

**Joint Space:** Describes the robot by its six joint angles. This is what we directly command. Two different joint configurations can place the end-effector at the same point in space.

**Cartesian Space:** Describes the end-effector position in X, Y, Z coordinates (mm) and orientation (RX, RY, RZ degrees). More intuitive for tasks but requires inverse kinematics to convert to joint angles.

**B-spline Interpolation:** Generates a smooth curve that passes near (but not exactly through) control points. Guarantees C² continuity — position, velocity, and acceleration are all smooth.

**Quintic Polynomial:** Generates a trajectory that passes exactly through each waypoint with zero velocity and zero acceleration at each one. Better for precise pick-and-place where the arm must stop at specific locations.

**CSV Logging:** Every command and resulting position is saved to a comma-separated values file for analysis in Python, Excel, or MATLAB.

---

## FILES AND LOCATIONS

| File | Location |
|------|----------|
| Main controller | `~/Desktop/PIPER_STUDENTS/piper_arm_hardware.py` |
| Plot script | `~/Desktop/PIPER_STUDENTS/plot_trajectory.py` |
| CSV log files | `~/Desktop/PIPER_CSV/` |
| RViz workspace | `~/piper_ws/` |

---

*TKM College of Engineering, Kollam — Mechanical Engineering Department*
