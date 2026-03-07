# 🤖 PIPER Robotic Arm Lab — TKM College of Engineering, Kollam

> **An educational robotics lab package for the PIPER 6-DOF robotic arm, developed for undergraduate students at TKM College of Engineering, Kollam.**

---

## 📦 Repository Contents

```
piper-arm-lab/
├── piper_arm_hardware.py        # Main hardware controller (full-featured)
├── examples/
│   ├── hello_piper.py           # Minimal "Hello World" — move one joint
│   ├── gripper_demo.py          # Open and close gripper
│   ├── waypoint_sequence.py     # Record and replay waypoints
│   └── trajectory_bspline.py   # B-spline trajectory example
├── utils/
│   └── plot_trajectory.py       # Plot CSV logs (joint + cartesian graphs)
├── docs/
│   ├── EXPERIMENT_MANUAL.md     # Full student experiment manual
│   ├── FUNCTION_REFERENCE.md   # Every function explained in detail
│   └── KNOWN_ISSUES.md         # Known bugs and workarounds
└── README.md                    # This file
```

---

## 🎯 What This Lab Covers

| Exercise | Mode | Skill |
|----------|------|-------|
| 1 — Basic Joint Control | Mode 1 | Understanding joint space |
| 2 — Sequence Programming | Mode 2 | Batch command execution |
| 3 — Trajectory Planning | Mode 3 | B-spline vs Quintic comparison |
| 4 — Data Analysis | CSV + Python | Plotting and comparing motion |

---

## ⚠️ Safety Rules — Read Before Anything Else

- ✅ Keep a **1-metre clear radius** around the arm at all times
- ✅ Always test sequences in **simulation (RViz)** before hardware
- ✅ Keep one hand near **E (Emergency Stop)** when running hardware
- ✅ Use **V1 or V2 speed** for first tests on hardware
- ✅ Never reach into the robot's workspace during motion
- ✅ Hardware runs require **instructor sign-off**

---

## 🚀 Quick Start

### Simulation (RViz / MoveIt)

```bash
# Terminal 1 — Launch RViz
cd ~/piper_ws
source devel/setup.bash
cd ~/piper_ws/src/piper_with_gripper_moveit/launch
roslaunch demo.launch

# Terminal 2 — Run controller
cd ~/Desktop/PIPER_STUDENTS
python3 piper_arm_hardware.py
```

### Hardware (After Instructor Approval)

```bash
# Bring up CAN bus
sudo ip link set can0 up type can bitrate 1000000

# Verify motors are communicating
candump can0        # Should see scrolling CAN frames

# Run controller
python3 piper_arm_hardware.py
```

---

## 🎮 Three Operating Modes

### Mode 1 — Command-by-Command
Enter one command at a time. Robot executes immediately.

```
CMD > AP10      # Move Joint 1 forward 10°
CMD > BN05      # Move Joint 2 back 5°
CMD > G10       # Open gripper 10mm
CMD > W         # Save waypoint
CMD > P         # Print end-effector position
```

### Mode 2 — Sequence Mode
Enter multiple commands on one line, separated by spaces.

```
SEQUENCE > AP10 BP20 W G10 W F10 W P
```

### Mode 3 — Trajectory Planning
Add waypoints, then execute a smooth interpolated path.

```
TRAJ > W          # Save waypoint 1
TRAJ > AP15 BP10
TRAJ > W          # Save waypoint 2
TRAJ > L          # List all waypoints
TRAJ > B          # Execute B-spline trajectory
```

---

## 📋 Command Quick Reference

| Command | Action |
|---------|--------|
| `AP10` | Joint 1 (A) Positive +10° |
| `BN05` | Joint 2 (B) Negative -5° |
| `G10` | Gripper open +10mm |
| `F10` | Gripper close -10mm |
| `W` | Add current position as waypoint |
| `L` | List waypoints |
| `CW` | Clear all waypoints |
| `P` | Print end-effector XYZ position |
| `V5` | Set speed to 5/10 |
| `E` | Emergency Stop |
| `R` | Resume after E-stop |
| `Z` | Reset arm to home (all zeros) |
| `S` | Save CSV log |
| `M` | Return to mode menu |
| `Q` | Quit |

Joints map: **A=1, B=2, C=3, D=4, E=5, F=6**

---

## 📊 Plotting Results

```bash
# Single file
python3 utils/plot_trajectory.py ~/Desktop/PIPER_CSV/robot_log_Mode3_Trajectory_20250210_143022.csv

# Compare two files (e.g. B-spline vs Quintic)
python3 utils/plot_trajectory.py \
  ~/Desktop/PIPER_CSV/robot_log_BSPLINE.csv \
  ~/Desktop/PIPER_CSV/robot_log_QUINTIC.csv
```

Output graphs:
- `*_joint_space.png` — Joint angles over time
- `*_cartesian_space.png` — End-effector XYZ path
- `*_smoothness.png` — Acceleration (smoothness metric)
- `*_comparison.png` — Side-by-side if two files given

---

## 🐛 Known Issues

| Feature | Status | Notes |
|---------|--------|-------|
| `draw_square()` | ❌ Not working | Linear move API mismatch — see `docs/KNOWN_ISSUES.md` |
| `safe_move_linear()` | ❌ Not working | `LinearMove` / `MovePose` not available in this SDK version |
| Joint control | ✅ Working | All 6 joints via `JointCtrl` |
| Gripper control | ✅ Working | Open/close via `GripperCtrl` |
| B-spline trajectory | ✅ Working | Use 5–10 points, 3–5s per point |
| Quintic trajectory | ✅ Working | Same recommendations as B-spline |
| CSV logging | ✅ Working | Saved to `~/Desktop/PIPER_CSV/` |

---

## 📁 Data Files

All CSV logs are saved to: `~/Desktop/PIPER_CSV/`

Each CSV contains:
```
Timestamp, Command, Joint1_deg … Joint6_deg, Gripper_mm,
EndEff_X_mm, EndEff_Y_mm, EndEff_Z_mm, EndEff_RX_deg, EndEff_RY_deg, EndEff_RZ_deg
```

---

## 📖 Documentation

- **[FUNCTION_REFERENCE.md](docs/FUNCTION_REFERENCE.md)** — Every class and function explained with examples
- **[EXPERIMENT_MANUAL.md](docs/EXPERIMENT_MANUAL.md)** — Step-by-step student lab guide
- **[KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md)** — Bugs, workarounds, and future fixes

---

## 🏫 Institution

**Department of Mechanical Engineering**
TKM College of Engineering, Kollam, Kerala — 691005

---

## 📄 License

MIT License — Free for educational use.
