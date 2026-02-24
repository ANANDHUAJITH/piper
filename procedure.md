# PiPER Robotic Arm – Operating Instructions

## ⚠️ SAFETY RULES
- Clear workspace (1 meter around robot)
- Test in simulation BEFORE hardware
- Press **E** for emergency stop if needed
- Never put hands near moving robot

---

## PART 1: SIMULATION (RViz/MoveIt)

### STEP 1 – Open Terminal
- Press **Super** key (Windows key)
- Type "Terminator" and press Enter

**Terminal shortcuts:**
```
Ctrl+Shift+C  →  Copy
Ctrl+Shift+V  →  Paste
Ctrl+C        →  Stop program
Ctrl+Shift+E  →  Split terminal (new pane)
```

---

### STEP 2 – Source Workspace (First Pane)
```bash
cd ~/piper_ws
source devel/setup.bash
```

---

### STEP 3 – Launch RViz (First Pane)
```bash
cd ~/piper_ws/src/piper_with_gripper_moveit/launch
roslaunch demo.launch
```

**Wait for RViz window to open** (shows 3D robot)

⚠️ **Keep this terminal running!** Don't close it.

---

### STEP 4 – Open New Terminal Pane
Press `Ctrl+Shift+E` (splits terminal vertically)

---

### STEP 5 – Navigate to Controller (New Pane)
```bash
cd ~/Desktop/PIPER_STUDENTS
```

---

### STEP 6 – Make Script Executable (New Pane)
```bash
chmod +x piper_moveit.py
```

---

### STEP 7 – Run Controller (New Pane)
```bash
python3 piper_moveit.py
```

You'll see mode selection menu:
```
Select mode (1-3) or Q to quit:
```

---

## THE THREE MODES

### MODE 1: Command-by-Command
- Type **one** command at a time
- Robot executes immediately
- Best for learning

**Example:**
```
CMD > AP10     # Joint 1 moves +10°
CMD > BP20     # Joint 2 moves +20°
CMD > W        # Save waypoint
```

---

### MODE 2: Sequence Mode
- Type **multiple** commands in one line (separated by spaces)
- Robot executes all in order
- Best for repeated patterns

**Example:**
```
SEQUENCE > AP10 BP20 W G05 W F10 W
```
(Moves joints, saves waypoints, opens/closes gripper)

---

### MODE 3: Trajectory Planning
- Create multiple waypoints
- Robot generates smooth path
- Best for smooth motion

**Example:**
```
TRAJ > W           # Save waypoint 1
TRAJ > AP30 BP20   # Move
TRAJ > W           # Save waypoint 2
TRAJ > L           # List waypoints
TRAJ > B           # Execute B-spline
```

---

## COMMAND REFERENCE

### Joint Movement
| Command | Action |
|---------|--------|
| AP10 | Joint 1 +10° |
| AN10 | Joint 1 -10° |
| BP20 | Joint 2 +20° |
| BN05 | Joint 2 -5° |
| CP15 | Joint 3 +15° |
| DP05 | Joint 4 +5° |
| EP30 | Joint 5 +30° |
| FP10 | Joint 6 +10° |

**Pattern:** `<Joint A-F><P or N><degrees>`

---

### Gripper
| Command | Action |
|---------|--------|
| G10 | Open +10mm |
| F10 | Close -10mm |

---

### Waypoints
| Command | Action |
|---------|--------|
| W | Add current position |
| L | List all waypoints |
| CW | Clear waypoints |
| V 3 | View waypoint #3 |
| D 2 | Delete waypoint #2 |

---

### Trajectory (Mode 3)
| Command | Action |
|---------|--------|
| B | Execute B-spline (smooth curves) |
| Q | Execute Quintic (smooth acceleration) |
| XD | Execute Direct (point-to-point) |

---

### Information
| Command | Action |
|---------|--------|
| P | Print position |

---

### Safety
| Command | Action |
|---------|--------|
| E | Emergency stop |
| R | Resume |
| Z | Reset to home (all joints 0°) |
| S | Save CSV file |
| M | Mode menu |
| Q | Quit |

---

## LEARNING SEQUENCE

### PHASE 1: Mode 1 Practice (15 min)

**Select Mode 1:**
```
Select mode (1-3) or Q to quit: 1
```

**Test each joint:**
```
CMD > AP10      # Watch Joint 1 move in RViz
CMD > AN10      # Watch it move back
CMD > BP20      # Watch Joint 2 move
CMD > BN20      # Watch it move back
```

**Test gripper:**
```
CMD > G10       # Open
CMD > F10       # Close
```

**Check position:**
```
CMD > P         # Shows X, Y, Z position
```

**Save waypoints:**
```
CMD > W         # Save current position
CMD > AP20 BP15
CMD > W         # Save new position
CMD > L         # List waypoints
```

**Test emergency stop:**
```
CMD > E         # Emergency stop
CMD > R         # Resume
```

**Return to menu:**
```
CMD > M
```

---

### PHASE 2: Mode 2 Practice (15 min)

**Select Mode 2:**
```
Select mode (1-3) or Q to quit: 2
```

**Test simple sequence:**
```
SEQUENCE > AP10 BP10 W P
```
(Moves joint 1, joint 2, saves waypoint, prints position)

**Pick-and-place pattern:**
```
SEQUENCE > AP20 BP15 W
SEQUENCE > F10 W
SEQUENCE > BN10 W
SEQUENCE > AN20 W
SEQUENCE > G10 W
SEQUENCE > L
```

**Return to menu:**
```
SEQUENCE > M
```

---

### PHASE 3: Mode 3 Trajectory (30 min)

**Select Mode 3:**
```
Select mode (1-3) or Q to quit: 3
```

**Create waypoints (20-30 total):**
```
TRAJ > W              # Waypoint 1
TRAJ > AP05 BP05 W    # Waypoint 2
TRAJ > AP05 BP05 W    # Waypoint 3
TRAJ > AP05 BP05 W    # Waypoint 4
TRAJ > AP05 BP05 W    # Waypoint 5
TRAJ > AP05 BN05 W    # Waypoint 6
TRAJ > AP05 BN05 W    # Waypoint 7
... (continue until 20-30 waypoints)
```

**List waypoints:**
```
TRAJ > L
```

**Execute B-spline:**
```
TRAJ > B
Number of interpolation points: 10
Seconds per point: 3
Execute? (y/n): y
```

**Save CSV:**
```
TRAJ > S
```
(Note the filename!)

**Clear and recreate same waypoints:**
```
TRAJ > CW
(Recreate the same 20-30 waypoints)
```

**Execute Quintic:**
```
TRAJ > Q
Number of interpolation points: 10
Seconds per point: 3
Execute? (y/n): y
```

**Save CSV:**
```
TRAJ > S
```
(Note this filename too!)

---

## PLOTTING GRAPHS

### Find CSV Files
```bash
cd ~/Desktop/PIPER_CSV
ls -lh
```

You'll see:
```
robot_log_Mode3_Trajectory_YYYYMMDD_HHMMSS.csv
```

---

### Plot Single File
```bash
cd ~/Desktop/PIPER_STUDENTS
python3 plot_trajectory.py ~/Desktop/PIPER_CSV/robot_log_Mode3_Trajectory_20250210_143022.csv
```
(Replace with your actual filename)

**Creates 3 graphs:**
- `*_joint_space.png` - Joint angles over time
- `*_cartesian_space.png` - End-effector X,Y,Z position
- `*_smoothness.png` - Acceleration (smoothness indicator)

---

### Compare B-spline vs Quintic
```bash
python3 plot_trajectory.py \
  ~/Desktop/PIPER_CSV/robot_log_BSPLINE.csv \
  ~/Desktop/PIPER_CSV/robot_log_QUINTIC.csv
```

**Creates comparison graph:**
- Shows both methods side-by-side

---

### View Graphs
```bash
cd ~/Desktop/PIPER_STUDENTS
eog *.png
```

Or copy to desktop:
```bash
cp *.png ~/Desktop/
```

---

## UNDERSTANDING GRAPHS

### Joint Space Graph
- Shows joint angles (degrees) over time
- One line per joint (6 total)
- Smooth curves = good motion

### Cartesian Space Graph
- Shows end-effector position X, Y, Z (mm)
- Top view shows path traced
- Different from joint space!

### Smoothness Graph
- Shows acceleration of each joint
- Lower values = smoother motion
- Compare B-spline vs Quintic here

### Comparison Graph
- Solid line = Method 1
- Dashed line = Method 2
- See which is smoother

---

## PART 2: HARDWARE (Only After Instructor Approval!)

### ⚠️ CHECKLIST
✅ Completed all simulation exercises
✅ Instructor approved
✅ Workspace clear (1 meter radius)
✅ Emergency stop accessible
✅ Team ready and alert

---

### STEP 1 – Source Workspace (New Terminal)
```bash
cd ~/piper_ws
source devel/setup.bash
```

---

### STEP 2 – Start CAN Interface
```bash
sudo ip link set can0 up type can bitrate 1000000
```
(Enter password when prompted)

**Check status:**
```bash
ip -details -statistics link show can0
```

**Look for:** `state ERROR-ACTIVE` (means CAN is OK)

---

### STEP 3 – Check Motor Communication (New Pane)
```bash
candump can0
```

You should see scrolling messages (motors talking)

Press `Ctrl+C` to stop

---

### STEP 4 – Run Hardware Controller (New Pane)
```bash
cd ~/Desktop/PIPER_STUDENTS
chmod +x piper_arm_hardware.py
python3 piper_arm_hardware.py
```

**When asked for CSV filename:**
```
Use default filename? (y/n): y
```

---

### STEP 5 – Safety Test

**Set slowest speed:**
```
Select mode: 1
CMD > V1
```

**Test emergency stop:**
```
CMD > E
CMD > R
```

**Test tiny movement:**
```
CMD > AP01
CMD > AN01
```

⚠️ **Watch carefully!** If anything wrong, press E immediately.

---

### STEP 6 – Run Verified Sequence

**Use sequences tested in simulation:**
```
Select mode: 2
SEQUENCE > AP10 BP10 W P
```

Or use Mode 3 with same waypoints from simulation.

⚠️ **Use 5 seconds per point for hardware!**

---

### STEP 7 – Save Data
```
CMD > S
```

---

### STEP 8 – Shutdown
```
CMD > Z          # Home position
CMD > Q          # Quit
```

**Bring down CAN:**
```bash
sudo ip link set can0 down
```

---

## COMPARING SIMULATION vs HARDWARE

### Plot Hardware Data
```bash
python3 plot_trajectory.py ~/Desktop/PIPER_CSV/robot_log_HARDWARE.csv
```

### Compare Simulation and Hardware
```bash
python3 plot_trajectory.py \
  ~/Desktop/PIPER_CSV/robot_log_SIMULATION.csv \
  ~/Desktop/PIPER_CSV/robot_log_HARDWARE.csv
```

Shows differences between simulation and real robot.

---

## QUICK TROUBLESHOOTING

| Problem | Solution |
|---------|----------|
| RViz doesn't open | Check: `source devel/setup.bash` |
| Robot doesn't move in RViz | Restart: `python3 piper_moveit.py` |
| CAN state DOWN | Run: `sudo ip link set can0 up...` |
| candump shows nothing | Check motor power and cables |
| Plot script error | Install: `pip3 install pandas matplotlib` |

---

## KEY CONCEPTS

**Joint Space:** Joint angles (degrees) - what you control directly

**Cartesian Space:** End-effector position (X,Y,Z in mm) - where robot reaches

**B-spline:** Smooth curves, faster

**Quintic:** Smoother acceleration, more controlled

**CSV File:** Saves all motion data for analysis

---

## SUMMARY

**You learned:**
1. Control robot in simulation
2. Three operating modes
3. Create waypoints and trajectories
4. Compare B-spline vs Quintic
5. Generate and analyze graphs
6. Operate hardware safely

**Files saved in:** `~/Desktop/PIPER_CSV/`

**To plot:** `python3 plot_trajectory.py <filename.csv>`

🎉 Done!
