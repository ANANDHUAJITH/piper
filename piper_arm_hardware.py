#!/usr/bin/env python3
# -*-coding:utf8-*-
# piper_arm_hardware.py
# -------------------------------------------------------
# Main PIPER robotic arm controller.
# Developed for TKM College of Engineering, Kollam.
#
# Three operating modes:
#   Mode 1 — Command-by-Command  (interactive, one at a time)
#   Mode 2 — Sequence Mode       (multiple commands per line)
#   Mode 3 — Trajectory Planning (waypoints + B-spline/Quintic)
#
# See docs/FUNCTION_REFERENCE.md for full documentation.
# See docs/KNOWN_ISSUES.md for known bugs (draw_square, linear move).
# -------------------------------------------------------

import time
import sys
import csv
import os
import numpy as np
from datetime import datetime
from piper_sdk import C_PiperInterface_V2


# ------------------------------------------------------------
# CSV Logger Class - Enhanced with Custom Save Path
# ------------------------------------------------------------
class RobotLogger:
    """
    Records robot state to a CSV file.

    Data is accumulated in memory (self.buffer) and only written
    to disk when save() is called or the logger is closed.
    This avoids file I/O overhead during motion.

    CSV columns:
      Timestamp, Command,
      Joint1_deg .. Joint6_deg, Gripper_mm,
      EndEff_X_mm .. EndEff_Z_mm, EndEff_RX_deg .. EndEff_RZ_deg
    """
    def __init__(self, mode_name):
        self.mode_name = mode_name
        self.filename = None
        self.buffer = []
        self.is_logging = False

        save_dir = os.path.expanduser("~/Desktop/PIPER_CSV")
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            print(f"📁 Created directory: {save_dir}")

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_name = f"robot_log_{mode_name}_{timestamp}.csv"

        print(f"\n📝 CSV Logging for {mode_name} mode")
        print(f"📂 Save location: {save_dir}")
        use_default = input(f"Use default filename '{default_name}'? (y/n): ").strip().lower()

        if use_default == 'y':
            self.filename = os.path.join(save_dir, default_name)
        else:
            custom_name = input("Enter CSV filename (without .csv): ").strip()
            if custom_name:
                self.filename = os.path.join(save_dir, f"{custom_name}.csv")
            else:
                self.filename = os.path.join(save_dir, default_name)

        print(f"📋 Logging will be saved to: {self.filename}")
        print("   (Press 'S' during operation to save)\n")
        self.is_logging = True

    def log(self, command, joints, gripper, pose):
        """Append one row to the in-memory buffer."""
        if not self.is_logging:
            return
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        joints_deg = [j/1000 for j in joints]
        gripper_mm = gripper/1000
        if pose:
            p = pose.end_pose
            pose_data = [
                p.X_axis/1000, p.Y_axis/1000, p.Z_axis/1000,
                p.RX_axis/1000, p.RY_axis/1000, p.RZ_axis/1000
            ]
        else:
            pose_data = [0, 0, 0, 0, 0, 0]
        row = [timestamp, command] + joints_deg + [gripper_mm] + pose_data
        self.buffer.append(row)

    def save(self):
        """Write buffer to disk as a CSV file."""
        if not self.buffer:
            print("⚠️  No data to save")
            return False
        try:
            with open(self.filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'Timestamp', 'Command',
                    'Joint1_deg', 'Joint2_deg', 'Joint3_deg',
                    'Joint4_deg', 'Joint5_deg', 'Joint6_deg',
                    'Gripper_mm',
                    'EndEff_X_mm', 'EndEff_Y_mm', 'EndEff_Z_mm',
                    'EndEff_RX_deg', 'EndEff_RY_deg', 'EndEff_RZ_deg'
                ])
                writer.writerows(self.buffer)
            print(f"✅ CSV saved: {self.filename} ({len(self.buffer)} entries)")
            return True
        except Exception as e:
            print(f"❌ Error saving CSV: {e}")
            return False

    def close(self):
        """Save remaining buffer on close."""
        if self.buffer:
            print(f"\n💾 Saving {len(self.buffer)} logged entries...")
            self.save()


# ------------------------------------------------------------
# B-Spline Trajectory Generator
# ------------------------------------------------------------
class BSplineTrajectory:
    """
    Generates smooth trajectories between waypoints.

    Two methods available:
      generate_trajectory()        — cubic B-spline (C2 smooth)
      generate_quintic_trajectory() — quintic polynomial (zero velocity at waypoints)

    Neither method moves the robot — they return a list of
    interpolated configurations. The caller is responsible for
    sending them to the robot.
    """

    @staticmethod
    def cubic_bspline_basis(t):
        """
        Cubic B-spline basis functions at parameter t in [0,1].
        Returns four blending weights (b0, b1, b2, b3).
        """
        t2 = t * t
        t3 = t2 * t
        b0 = (1 - t)**3 / 6
        b1 = (3*t3 - 6*t2 + 4) / 6
        b2 = (-3*t3 + 3*t2 + 3*t + 1) / 6
        b3 = t3 / 6
        return b0, b1, b2, b3

    @staticmethod
    def generate_trajectory(waypoints, num_points=50):
        """
        Generate B-spline trajectory through waypoints.

        waypoints  : list of [J1,J2,J3,J4,J5,J6,Gripper_mm] in degrees/mm
        num_points : number of output interpolated points

        Returns list of interpolated configurations.
        Note: curve passes NEAR waypoints, not exactly through them.
        """
        if len(waypoints) < 2:
            return waypoints
        waypoints = np.array(waypoints)
        n_waypoints = len(waypoints)
        if n_waypoints < 4:
            padded = [waypoints[0]]
            padded.extend(waypoints)
            padded.append(waypoints[-1])
            waypoints = np.array(padded)
            n_waypoints = len(waypoints)
        trajectory = []
        n_segments = n_waypoints - 3
        points_per_segment = max(1, num_points // n_segments)
        for seg in range(n_segments):
            for i in range(points_per_segment):
                t = i / points_per_segment
                b0, b1, b2, b3 = BSplineTrajectory.cubic_bspline_basis(t)
                point = (b0 * waypoints[seg] +
                         b1 * waypoints[seg + 1] +
                         b2 * waypoints[seg + 2] +
                         b3 * waypoints[seg + 3])
                trajectory.append(point.tolist())
        trajectory.append(waypoints[-1].tolist())
        return trajectory

    @staticmethod
    def generate_quintic_trajectory(waypoints, num_points=50):
        """
        Generate quintic polynomial trajectory.

        Uses s(t) = 10t³ - 15t⁴ + 6t⁵ which guarantees
        zero velocity and zero acceleration at t=0 and t=1.
        This means the robot decelerates to a stop at each waypoint.

        waypoints  : list of [J1,J2,J3,J4,J5,J6,Gripper_mm] in degrees/mm
        num_points : number of output interpolated points

        Returns list of interpolated configurations.
        Note: curve passes EXACTLY through every waypoint.
        """
        if len(waypoints) < 2:
            return waypoints
        waypoints = np.array(waypoints)
        trajectory = []
        n_segments = len(waypoints) - 1
        points_per_segment = max(1, num_points // n_segments)
        for seg in range(n_segments):
            start = waypoints[seg]
            end = waypoints[seg + 1]
            for i in range(points_per_segment):
                t = i / points_per_segment
                s = 10*t**3 - 15*t**4 + 6*t**5
                point = start + s * (end - start)
                trajectory.append(point.tolist())
        trajectory.append(waypoints[-1].tolist())
        return trajectory


# ------------------------------------------------------------
# Global Waypoints Storage
# ------------------------------------------------------------
class WaypointManager:
    """
    Stores waypoints in memory during a session.
    Each waypoint: [J1, J2, J3, J4, J5, J6, Gripper_mm] in degrees/mm.
    Waypoints are lost on program exit unless saved via the shutdown prompt.
    """
    def __init__(self):
        self.waypoints = []

    def add_current(self, joints, gripper):
        """Add current robot position as waypoint. Converts from millidegrees."""
        waypoint = [j/1000 for j in joints]
        waypoint.append(gripper/1000)
        self.waypoints.append(waypoint)
        return len(self.waypoints)

    def add_manual(self, values):
        """Add waypoint with manually specified values (7 floats)."""
        if len(values) != 7:
            return False
        self.waypoints.append(values)
        return True

    def list_all(self):
        """Print all waypoints to terminal."""
        if not self.waypoints:
            print("📭 No waypoints stored")
            return
        print(f"\n📍 {len(self.waypoints)} waypoints:")
        for i, wp in enumerate(self.waypoints, 1):
            joints_str = ', '.join([f'{j:6.1f}°' for j in wp[:6]])
            print(f"  {i}: [{joints_str}] Gripper: {wp[6]:6.1f}mm")
        print()

    def clear(self):
        self.waypoints.clear()

    def get_all(self):
        return self.waypoints.copy()

    def delete(self, index):
        if 0 <= index < len(self.waypoints):
            return self.waypoints.pop(index)
        return None


# ------------------------------------------------------------
# Emergency Stop State
# ------------------------------------------------------------
class EmergencyStop:
    """
    Software emergency stop flag.
    Checked before every robot command. Does not physically
    interrupt a command already sent to the motor controllers.
    Use the physical e-stop button for true hardware interrupt.
    """
    def __init__(self):
        self.is_stopped = False

    def activate(self):
        self.is_stopped = True
        print("\n🚨 EMERGENCY STOP ACTIVATED! 🚨")
        print("All motion halted. Press R to resume.\n")

    def deactivate(self):
        self.is_stopped = False
        print("\n✅ Emergency stop cleared. System resumed.\n")

    def check(self):
        return self.is_stopped


# ------------------------------------------------------------
# Speed Controller
# ------------------------------------------------------------
class SpeedController:
    """
    Wraps MotionCtrl_2 to provide a 1-10 speed scale.
    Speed 1 = slowest (safest), Speed 10 = maximum.
    Default is 8.
    """
    def __init__(self, piper, default_speed=3):
        self.piper = piper
        self.current_speed = default_speed
        self.min_speed = 1
        self.max_speed = 10

    def set_speed(self, speed):
        if speed < self.min_speed or speed > self.max_speed:
            print(f"❌ Speed must be between {self.min_speed} and {self.max_speed}")
            return False
        self.current_speed = speed
        self.piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        descriptions = {
            1: "Very Slow (Maximum Safety)", 2: "Slow",
            3: "Slow-Medium (Default)", 4: "Medium-Slow",
            5: "Medium", 6: "Medium-Fast", 7: "Fast",
            8: "Very Fast", 9: "Very Fast+", 10: "Maximum Speed"
        }
        print(f"⚡ Speed set to {speed}/10 - {descriptions.get(speed, '')}")
        return True

    def get_speed(self):
        return self.current_speed

    def print_speed_info(self):
        print(f"\n⚡ Current Speed: {self.current_speed}/10")
        print("   1-3: Slow | 4-6: Medium | 7-8: Fast | 9-10: Very Fast")
        print("   Use: V<number> (e.g., V5, V8, V10)\n")


# ------------------------------------------------------------
# Safe Joint & Gripper Readers
# ------------------------------------------------------------
def safe_read_joints(piper):
    """Read all 6 joint angles in millidegrees. Returns list or None."""
    msg = piper.GetArmJointMsgs()
    if not msg:
        return None
    js = msg.joint_state
    return [js.joint_1, js.joint_2, js.joint_3,
            js.joint_4, js.joint_5, js.joint_6]


def safe_read_gripper(piper):
    """Read gripper angle in millidegrees. Returns value or None."""
    msg = piper.GetArmGripperMsgs()
    if not msg:
        return None
    return msg.gripper_state.grippers_angle


# ------------------------------------------------------------
# Print End-Effector Position
# ------------------------------------------------------------
def print_position(piper):
    """Print current end-effector Cartesian position (mm and degrees)."""
    pose = piper.GetArmEndPoseMsgs()
    if pose:
        p = pose.end_pose
        print("\n📍 Current End-Effector Position:")
        print(f"   X  = {p.X_axis/1000:8.2f} mm")
        print(f"   Y  = {p.Y_axis/1000:8.2f} mm")
        print(f"   Z  = {p.Z_axis/1000:8.2f} mm")
        print(f"   RX = {p.RX_axis/1000:8.2f}°")
        print(f"   RY = {p.RY_axis/1000:8.2f}°")
        print(f"   RZ = {p.RZ_axis/1000:8.2f}°\n")
    else:
        print("⚠️  Could not read position\n")


# ------------------------------------------------------------
# SDK Compatibility Wrapper for Linear Move
# NOTE: This function does NOT work with the current SDK version.
#       See docs/KNOWN_ISSUES.md for details and workaround.
# ------------------------------------------------------------
def safe_move_linear(piper, x, y, z, rx, ry, rz, speed):
    """
    Attempt Cartesian linear move. NOT WORKING — SDK missing LinearMove/MovePose.
    See docs/KNOWN_ISSUES.md.
    """
    if hasattr(piper, "LinearMove"):
        return piper.LinearMove(x, y, z, rx, ry, rz, speed)
    if hasattr(piper, "MovePose"):
        return piper.MovePose(x, y, z, rx, ry, rz, speed)
    print("❌ No linear move in SDK version! See docs/KNOWN_ISSUES.md")
    return None


# ------------------------------------------------------------
# Draw a small square in YZ plane
# NOTE: This function does NOT work — depends on safe_move_linear().
#       See docs/KNOWN_ISSUES.md for workaround.
# ------------------------------------------------------------
def draw_square(piper, side=50):
    """
    Draw a 50mm square in YZ plane. NOT WORKING — see docs/KNOWN_ISSUES.md.
    Workaround: use joint-space waypoints in Mode 3 with XD or B commands.
    """
    pose_msg = piper.GetArmEndPoseMsgs()
    if not pose_msg:
        print("⚠️ Can't read end-effector pose.")
        return
    p = pose_msg.end_pose
    X, Y, Z = p.X_axis, p.Y_axis, p.Z_axis
    RX, RY, RZ = p.RX_axis, p.RY_axis, p.RZ_axis
    print("Drawing a square... (⚠️ May not work — see KNOWN_ISSUES.md)")
    points = [
        (X, Y,        Z),
        (X, Y + side, Z),
        (X, Y + side, Z + side),
        (X, Y,        Z + side),
        (X, Y,        Z),
    ]
    for (x, y, z) in points:
        safe_move_linear(piper, x, y, z, RX, RY, RZ, 10)
        time.sleep(0.05)
    print("Square attempt complete.\n")


# ------------------------------------------------------------
# Execute Single Command
# ------------------------------------------------------------
def execute_command(cmd, piper, joints, gripper, logger, waypoint_mgr, e_stop, speed_ctrl):
    """
    Parse and execute a single command string.

    This is the central dispatcher used by all three modes.

    Parameters:
      cmd         — command string (e.g. "AP10", "G05", "W", "E")
      piper       — C_PiperInterface_V2 instance
      joints      — current joint state in millidegrees (list of 6)
      gripper     — current gripper position in millidegrees
      logger      — RobotLogger instance for this mode
      waypoint_mgr— WaypointManager (shared across modes)
      e_stop      — EmergencyStop instance
      speed_ctrl  — SpeedController instance

    Returns: (joints, gripper, should_continue)
      should_continue is False only when Q (quit) is received.
    """
    GRIP_MIN = 0
    GRIP_MAX = 80000
    GRIP_EFFORT = 500

    cmd = cmd.strip().upper()

    if cmd == "E":
        e_stop.activate()
        return joints, gripper, True

    if e_stop.check():
        if cmd == "R":
            e_stop.deactivate()
        else:
            print("⚠️  System in EMERGENCY STOP. Press R to resume.")
        return joints, gripper, True

    if cmd.startswith("V"):
        if len(cmd) == 1:
            speed_ctrl.print_speed_info()
        else:
            try:
                speed_ctrl.set_speed(int(cmd[1:]))
            except ValueError:
                print("❌ Invalid speed. Use V1 to V10")
        return joints, gripper, True

    if cmd == "S":
        logger.save()
        return joints, gripper, True

    if cmd in ("P", "C"):
        print_position(piper)
        return joints, gripper, True

    if cmd == "W":
        cj = safe_read_joints(piper)
        cg = safe_read_gripper(piper)
        if cj and cg is not None:
            n = waypoint_mgr.add_current(cj, cg)
            print(f"✅ Waypoint {n} added: [{', '.join([f'{j/1000:.1f}°' for j in cj])}]  Gripper: {cg/1000:.1f}mm")
        else:
            print("❌ Failed to read current position")
        return joints, gripper, True

    if cmd == "L":
        waypoint_mgr.list_all()
        return joints, gripper, True

    if cmd == "CW":
        waypoint_mgr.clear()
        print("🗑️  All waypoints cleared")
        return joints, gripper, True

    if cmd == "Q":
        return joints, gripper, False

    if cmd == "Z":
        print("⚠️  Resetting arm to HOME position (all joints to zero)...")
        if input("Are you sure? (y/n): ").strip().lower() != 'y':
            print("Reset cancelled.")
            return joints, gripper, True
        joints = [0, 0, 0, 0, 0, 0]
        gripper = 0
        piper.JointCtrl(*joints)
        piper.GripperCtrl(gripper, GRIP_EFFORT, 0x01, 0)
        time.sleep(0.5)
        logger.log(cmd, joints, gripper, piper.GetArmEndPoseMsgs())
        print("✅ Arm reset to HOME complete")
        return joints, gripper, True

    if cmd == "SQ":
        draw_square(piper)
        logger.log(cmd, joints, gripper, piper.GetArmEndPoseMsgs())
        return joints, gripper, True

    # Joint movement: AP10, BN05, CP30, etc.
    if len(cmd) >= 3 and cmd[0] in "ABCDEF" and cmd[1] in "PN":
        joint_index = ord(cmd[0]) - ord('A')
        direction = 1 if cmd[1] == "P" else -1
        try:
            amount_deg = int(cmd[2:])
        except:
            print("❌ Invalid numeric value.")
            return joints, gripper, True
        joints[joint_index] += direction * amount_deg * 1000
        print(f"Joint {joint_index+1} = {joints[joint_index]/1000:.2f}°")
        piper.JointCtrl(*joints)
        time.sleep(0.3)
        logger.log(cmd, joints, gripper, piper.GetArmEndPoseMsgs())
        return joints, gripper, True

    # Gripper open: G10 = open 10mm
    if cmd.startswith("G") and cmd[1:].isdigit():
        mm = int(cmd[1:])
        gripper = min(GRIP_MAX, gripper + mm * 1000)
        print(f"Gripper = {gripper/1000:.1f} mm")
        piper.GripperCtrl(gripper, GRIP_EFFORT, 0x01, 0)
        time.sleep(0.3)
        logger.log(cmd, joints, gripper, piper.GetArmEndPoseMsgs())
        return joints, gripper, True

    # Gripper close: F10 = close 10mm
    if cmd.startswith("F") and cmd[1:].isdigit():
        mm = int(cmd[1:])
        gripper = max(GRIP_MIN, gripper - mm * 1000)
        print(f"Gripper = {gripper/1000:.1f} mm")
        piper.GripperCtrl(gripper, GRIP_EFFORT, 0x01, 0)
        time.sleep(0.3)
        logger.log(cmd, joints, gripper, piper.GetArmEndPoseMsgs())
        return joints, gripper, True

    print("❌ Unknown command.\n")
    return joints, gripper, True


# ------------------------------------------------------------
# MODE 1: Command-by-Command
# ------------------------------------------------------------
def mode_command_by_command(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl):
    print("\n" + "="*60)
    print("🎮 MODE 1: Command-by-Command")
    print("="*60)
    logger = RobotLogger("Mode1_Interactive")
    print("""
Commands:
  Joint:     AP10, BN05 (A-F = joints 1-6, P/N = +/-, then degrees)
  Gripper:   G10 (open), F10 (close)
  Position:  P  Speed: V / V1-V10
  Waypoints: W (add), L (list), CW (clear)
  Safety:    E (e-stop), R (resume), Z (home)
  Other:     S (save CSV), M (mode menu), Q (quit)
""")
    try:
        while True:
            cmd = input("CMD > ").strip().upper()
            if cmd == "M":
                if logger.buffer:
                    if input(f"Save {len(logger.buffer)} entries before switching? (y/n): ").strip().lower() == 'y':
                        logger.save()
                logger.close()
                return joints, gripper
            joints, gripper, cont = execute_command(
                cmd, piper, joints, gripper, logger, waypoint_mgr, e_stop, speed_ctrl)
            if not cont:
                logger.close()
                return joints, gripper
    except KeyboardInterrupt:
        print("\n⚠️  Mode interrupted")
        logger.close()
        return joints, gripper


# ------------------------------------------------------------
# MODE 2: Sequence Mode
# ------------------------------------------------------------
def mode_sequence(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl):
    print("\n" + "="*60)
    print("📋 MODE 2: Sequence Mode")
    print("="*60)
    logger = RobotLogger("Mode2_Sequence")
    print("""
Enter commands separated by spaces.
Example: AP10 BP20 G05 F10 W P
M = mode menu, Q = quit
""")
    try:
        while True:
            seq = input("SEQUENCE > ").strip()
            if seq.upper() == "M":
                if logger.buffer and input(f"Save {len(logger.buffer)} entries? (y/n): ").strip().lower() == 'y':
                    logger.save()
                logger.close()
                return joints, gripper
            if seq.upper() == "Q":
                if logger.buffer and input(f"Save {len(logger.buffer)} entries? (y/n): ").strip().lower() == 'y':
                    logger.save()
                logger.close()
                return joints, gripper
            commands = seq.split()
            if not commands:
                print("❌ Empty sequence.\n")
                continue
            print(f"\n▶️  Executing {len(commands)} commands...")
            for i, cmd in enumerate(commands, 1):
                print(f"\n[{i}/{len(commands)}] {cmd}")
                joints, gripper, cont = execute_command(
                    cmd, piper, joints, gripper, logger, waypoint_mgr, e_stop, speed_ctrl)
                if not cont:
                    logger.close()
                    return joints, gripper
                if e_stop.check():
                    print("⚠️  Sequence halted by emergency stop")
                    break
                time.sleep(0.1)
            print("\n✅ Sequence complete!\n")
    except KeyboardInterrupt:
        print("\n⚠️  Mode interrupted")
        logger.close()
        return joints, gripper


# ------------------------------------------------------------
# MODE 3: Trajectory Planning
# ------------------------------------------------------------
def mode_trajectory_planning(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl):
    print("\n" + "="*60)
    print("🎯 MODE 3: Trajectory Planning (With Gripper Control)")
    print("="*60)
    logger = RobotLogger("Mode3_Trajectory")
    print("""
W           - Add current position as waypoint
W J1,...,G  - Add waypoint manually (7 values: 6 joints + gripper_mm)
L           - List waypoints
CW          - Clear all waypoints
D <n>       - Delete waypoint n
V <n>       - View waypoint n
P           - Print current position
V / V1-V10  - Speed control

B           - Execute B-spline trajectory
Q           - Execute Quintic trajectory
XD          - Execute Direct waypoint sequence

Z / E / R / S / M / X  - home / e-stop / resume / save / menu / exit
""")
    try:
        while True:
            cmd = input("TRAJ > ").strip()
            if not cmd:
                continue
            cu = cmd.upper()

            if cu in ("M", "X"):
                if logger.buffer and input(f"Save {len(logger.buffer)} entries? (y/n): ").strip().lower() == 'y':
                    logger.save()
                logger.close()
                return joints, gripper

            if cu == "E":
                e_stop.activate(); continue
            if e_stop.check():
                if cu == "R": e_stop.deactivate()
                else: print("⚠️  EMERGENCY STOP active. Press R to resume.")
                continue

            if cu == "S":
                logger.save(); continue
            if cu == "P":
                print_position(piper)
                cg = safe_read_gripper(piper)
                if cg is not None: print(f"   Gripper: {cg/1000:.1f}mm\n")
                continue

            # Speed and waypoint view share "V"
            if cu.startswith("V"):
                parts = cmd.strip().split()
                if len(parts) == 2 and parts[1].isdigit():
                    idx = int(parts[1]) - 1
                    wps = waypoint_mgr.get_all()
                    if 0 <= idx < len(wps):
                        wp = wps[idx]
                        print(f"Waypoint {idx+1}: [{', '.join([f'{j:.1f}°' for j in wp[:6]])}]  Grip={wp[6]:.1f}mm")
                    else:
                        print("❌ Invalid waypoint number")
                elif cu == "V":
                    speed_ctrl.print_speed_info()
                elif cu[1:].isdigit():
                    try: speed_ctrl.set_speed(int(cu[1:]))
                    except ValueError: print("❌ Invalid speed")
                continue

            if cu == "W":
                cj = safe_read_joints(piper); cg = safe_read_gripper(piper)
                if cj and cg is not None:
                    n = waypoint_mgr.add_current(cj, cg)
                    print(f"✅ Waypoint {n}: [{', '.join([f'{j/1000:.1f}°' for j in cj])}]  Grip={cg/1000:.1f}mm")
                else:
                    print("❌ Could not read position")
                continue

            if cu.startswith("W "):
                try:
                    vals = [float(v) for v in cmd[2:].strip().split(',')]
                    if len(vals) != 7: raise ValueError
                    waypoint_mgr.add_manual(vals)
                    print(f"✅ Waypoint {len(waypoint_mgr.waypoints)} added")
                except:
                    print("❌ Format: W J1,J2,J3,J4,J5,J6,Gripper_mm")
                continue

            if cu == "L": waypoint_mgr.list_all(); continue
            if cu == "CW": waypoint_mgr.clear(); print("🗑️  Cleared"); continue
            if cu == "Z":
                print("⚠️  Reset to home?")
                if input("Confirm? (y/n): ").strip().lower() == 'y':
                    joints = [0,0,0,0,0,0]; gripper = 0
                    piper.JointCtrl(*joints); piper.GripperCtrl(0, 500, 0x01, 0)
                    time.sleep(0.5); print("✅ Home")
                continue

            if cu.startswith("D "):
                try:
                    removed = waypoint_mgr.delete(int(cmd[2:].strip()) - 1)
                    print(f"🗑️  Deleted: {removed}" if removed else "❌ Invalid index")
                except: print("❌ Invalid number")
                continue

            # ---- Trajectory execution helper ----
            def run_trajectory(traj, label):
                nonlocal joints, gripper
                if not traj: return
                total_est = len(traj) * time_per_point
                print(f"\n📈 {len(traj)} points | ~{total_est:.1f}s estimated")
                if input(f"Execute {label}? (y/n): ").strip().lower() != 'y':
                    return
                print(f"▶️  Executing {label}...\n")
                start = time.time()
                for i, pt in enumerate(traj):
                    if e_stop.check(): print("⚠️  Halted"); break
                    jc = [int(j*1000) for j in pt[:6]]
                    gc = int(pt[6]*1000)
                    piper.JointCtrl(*jc)
                    piper.GripperCtrl(gc, 500, 0x01, 0)
                    logger.log(f"{label}_{i}", jc, gc, piper.GetArmEndPoseMsgs())
                    time.sleep(time_per_point)
                    cp = safe_read_joints(piper); cg2 = safe_read_gripper(piper)
                    pct = 100*(i+1)//len(traj)
                    el = time.time()-start
                    if cp and cg2 is not None:
                        print(f"  [{i+1}/{len(traj)}] ({pct:3d}%) {el:5.1f}s  J1={cp[0]/1000:6.1f}° Grip={cg2/1000:5.1f}mm")
                    else:
                        print(f"  [{i+1}/{len(traj)}] ({pct:3d}%) {el:5.1f}s")
                if not e_stop.check():
                    joints = jc; gripper = gc
                    print(f"\n✅ {label} complete in {time.time()-start:.1f}s!")

            if cu == "B":
                wps = waypoint_mgr.get_all()
                if len(wps) < 2: print("❌ Need at least 2 waypoints"); continue
                try: num_points = int(input("Interpolation points (default 5): ").strip() or "5")
                except: num_points = 5
                try: time_per_point = float(input("Seconds per point (default 5): ").strip() or "5")
                except: time_per_point = 5.0
                traj = BSplineTrajectory.generate_trajectory(wps, num_points=num_points)
                run_trajectory(traj, "BSPLINE")
                continue

            if cu == "Q":
                wps = waypoint_mgr.get_all()
                if len(wps) < 2: print("❌ Need at least 2 waypoints"); continue
                try: num_points = int(input("Interpolation points (default 5): ").strip() or "5")
                except: num_points = 5
                try: time_per_point = float(input("Seconds per point (default 5): ").strip() or "5")
                except: time_per_point = 5.0
                traj = BSplineTrajectory.generate_quintic_trajectory(wps, num_points=num_points)
                run_trajectory(traj, "QUINTIC")
                continue

            if cu.startswith("XD"):
                wps = waypoint_mgr.get_all()
                if len(wps) < 2: print("❌ Need at least 2 waypoints"); continue
                try: dwell = float(input("Dwell time per waypoint seconds (default 1.0): ").strip() or "1.0")
                except: dwell = 1.0
                if input(f"Execute direct sequence through {len(wps)} waypoints? (y/n): ").strip().lower() != 'y':
                    continue
                print("▶️  Direct execution...\n")
                for i, wp in enumerate(wps, 1):
                    if e_stop.check(): print("⚠️  Halted"); break
                    jc = [int(j*1000) for j in wp[:6]]; gc = int(wp[6]*1000)
                    print(f"Waypoint {i}/{len(wps)}: [{', '.join([f'{j:.1f}°' for j in wp[:6]])}]  Grip={wp[6]:.1f}mm")
                    piper.JointCtrl(*jc); piper.GripperCtrl(gc, 500, 0x01, 0)
                    logger.log(f"DIRECT_{i}", jc, gc, piper.GetArmEndPoseMsgs())
                    time.sleep(dwell)
                if not e_stop.check():
                    joints = jc; gripper = gc
                    print("✅ Direct sequence complete!")
                continue

            print("❌ Unknown command. Type M for menu.\n")

    except KeyboardInterrupt:
        print("\n⚠️  Mode interrupted")
        logger.close()
        return joints, gripper


# ------------------------------------------------------------
# Mode Selection Menu
# ------------------------------------------------------------
def show_mode_menu():
    print("\n" + "="*60)
    print("                 MODE SELECTION MENU")
    print("="*60)
    print("""
  1. Command-by-Command  — one command at a time
  2. Sequence Mode       — multiple commands per line
  3. Trajectory Planning — waypoints + smooth interpolation

  Q  — Quit
""")
    return input("Select (1-3 or Q): ").strip()


# ------------------------------------------------------------
# MAIN PROGRAM
# ------------------------------------------------------------
def main():
    print("\n" + "="*60)
    print("   PIPER ROBOT CONTROL — TKM College of Engineering")
    print("="*60)

    waypoint_mgr = WaypointManager()
    e_stop = EmergencyStop()

    piper = C_PiperInterface_V2(
        "can0",
        start_sdk_joint_limit=True,
        start_sdk_gripper_limit=True
    )
    piper.ConnectPort()

    print("\n🔌 Connecting to robot...")
    while not piper.EnablePiper():
        time.sleep(0.01)
    print("✅ Piper Enabled.")

    print("\n⚙️  Applying safety settings...")
    speed_ctrl = SpeedController(piper, default_speed=8)
    speed_ctrl.set_speed(8)
    piper.CrashProtectionConfig(1, 1, 1, 1, 1, 1)
    print("✅ Safety settings applied (speed 8/10, max crash protection)")

    joints = safe_read_joints(piper) or [0,0,0,0,0,0]
    gripper = safe_read_gripper(piper)
    if gripper is None: gripper = 0

    print(f"\n📊 Initial State:")
    print(f"   Joints:  {[f'{j/1000:.1f}°' for j in joints]}")
    print(f"   Gripper: {gripper/1000:.1f} mm")
    print(f"   Speed:   {speed_ctrl.get_speed()}/10")
    print(f"\n📂 CSV files will be saved to: ~/Desktop/PIPER_CSV/")

    try:
        while True:
            choice = show_mode_menu()
            if choice.upper() == "Q":
                print("\n👋 Exiting..."); break
            elif choice == "1":
                joints, gripper = mode_command_by_command(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl)
            elif choice == "2":
                joints, gripper = mode_sequence(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl)
            elif choice == "3":
                joints, gripper = mode_trajectory_planning(piper, joints, gripper, waypoint_mgr, e_stop, speed_ctrl)
            else:
                print("❌ Invalid selection. Choose 1-3 or Q.")
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n⚠️  Program interrupted by user")

    finally:
        print("\n" + "="*60)
        print("                  SHUTDOWN")
        print("="*60)
        if waypoint_mgr.waypoints:
            print(f"\n📍 {len(waypoint_mgr.waypoints)} waypoints in memory.")
            if input("Save waypoints to file? (y/n): ").strip().lower() == 'y':
                save_dir = os.path.expanduser("~/Desktop/PIPER_CSV")
                fn = os.path.join(save_dir, f"waypoints_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt")
                try:
                    with open(fn, 'w') as f:
                        f.write("# PIPER Robot Waypoints\n")
                        f.write("# Format: [J1, J2, J3, J4, J5, J6, Gripper_mm] — degrees/mm\n\n")
                        for i, wp in enumerate(waypoint_mgr.waypoints, 1):
                            f.write(f"Waypoint {i}: {wp}\n")
                    print(f"✅ Saved: {fn}")
                except Exception as e:
                    print(f"❌ Error: {e}")
        print("\n✅ Shutdown complete. Goodbye!")


if __name__ == "__main__":
    main()
