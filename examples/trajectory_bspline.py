#!/usr/bin/env python3
# trajectory_bspline.py
# -------------------------------------------------------
# Standalone B-spline trajectory example.
# Shows how to generate and execute a smooth trajectory
# without the full Mode 3 interactive UI.
#
# Useful for:
#   - Understanding how BSplineTrajectory works
#   - Running a fixed trajectory in an automated script
#   - Comparing B-spline vs Quintic in the same file
# -------------------------------------------------------

import time
import numpy as np
from piper_sdk import C_PiperInterface_V2

# ----------------------------------------------------------
# B-Spline Trajectory Generator (self-contained copy)
# ----------------------------------------------------------
class BSplineTrajectory:
    """
    Cubic B-spline interpolation between waypoints.

    How it works:
      - Takes N waypoints as input
      - Generates a smooth curve with many intermediate points
      - The curve passes NEAR (not exactly through) each waypoint
      - Velocity and acceleration are continuous (C2 continuity)
    """

    @staticmethod
    def cubic_bspline_basis(t):
        """
        Compute the 4 cubic B-spline basis functions at parameter t.

        t    : float in [0, 1] — position along one curve segment
        Returns: (b0, b1, b2, b3) — blending weights for 4 control points
        """
        t2 = t * t
        t3 = t2 * t
        b0 = (1 - t)**3 / 6
        b1 = (3*t3 - 6*t2 + 4) / 6
        b2 = (-3*t3 + 3*t2 + 3*t + 1) / 6
        b3 = t3 / 6
        return b0, b1, b2, b3

    @staticmethod
    def generate(waypoints, num_points=10):
        """
        Generate a B-spline trajectory.

        waypoints  : list of [J1, J2, J3, J4, J5, J6, Gripper_mm]
                     values in degrees / mm
        num_points : total number of points in the output trajectory

        Returns: list of interpolated configurations (same format as input)
        """
        wp = np.array(waypoints)
        n = len(wp)

        if n < 2:
            return waypoints

        # Pad endpoints if fewer than 4 waypoints
        if n < 4:
            wp = np.vstack([wp[0], wp, wp[-1]])
            n = len(wp)

        trajectory = []
        n_segments = n - 3
        pts_per_seg = max(1, num_points // n_segments)

        for seg in range(n_segments):
            for i in range(pts_per_seg):
                t = i / pts_per_seg
                b0, b1, b2, b3 = BSplineTrajectory.cubic_bspline_basis(t)
                pt = (b0 * wp[seg] + b1 * wp[seg+1] +
                      b2 * wp[seg+2] + b3 * wp[seg+3])
                trajectory.append(pt.tolist())

        trajectory.append(wp[-1].tolist())   # Ensure last point is included
        return trajectory


# ----------------------------------------------------------
# Define waypoints here
# Format: [J1, J2, J3, J4, J5, J6, Gripper_mm]  (degrees / mm)
# ----------------------------------------------------------
WAYPOINTS = [
    [  0,   0,   0,   0,   0,   0,   0],   # Home
    [ 10,   5,   0,   0,   0,   0,  10],
    [ 20,  10,   5,   0,   0,   0,  20],
    [ 25,  15,  10,   0,   0,   0,  40],
    [ 20,  10,   5,   0,   0,   0,  20],
    [ 10,   5,   0,   0,   0,   0,   0],
    [  0,   0,   0,   0,   0,   0,   0],   # Back to home
]

NUM_INTERP_POINTS = 10       # Number of interpolated points
TIME_PER_POINT    = 3.0      # Seconds per interpolated point
GRIP_EFFORT       = 500
SPEED             = 3        # 1–10

# ----------------------------------------------------------
# Connect
# ----------------------------------------------------------
piper = C_PiperInterface_V2("can0",
                             start_sdk_joint_limit=True,
                             start_sdk_gripper_limit=True)
piper.ConnectPort()

print("Connecting to PIPER arm...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Connected!")

piper.MotionCtrl_2(0x01, 0x01, SPEED, 0x00)
print(f"Speed: {SPEED}/10\n")

# ----------------------------------------------------------
# Generate trajectory
# ----------------------------------------------------------
print(f"Generating B-spline trajectory from {len(WAYPOINTS)} waypoints...")
trajectory = BSplineTrajectory.generate(WAYPOINTS, num_points=NUM_INTERP_POINTS)
total_time = len(trajectory) * TIME_PER_POINT
print(f"Generated {len(trajectory)} points")
print(f"Estimated execution time: {total_time:.1f}s ({total_time/60:.1f} min)\n")

confirm = input("Execute trajectory? (y/n): ").strip().lower()
if confirm != 'y':
    print("Cancelled.")
    exit()

# ----------------------------------------------------------
# Execute trajectory
# ----------------------------------------------------------
print("\nExecuting B-spline trajectory...")
print("Press Ctrl+C to stop\n")

start = time.time()

for i, point in enumerate(trajectory):
    joints_cmd  = [int(j * 1000) for j in point[:6]]
    gripper_cmd = int(point[6] * 1000)

    piper.JointCtrl(*joints_cmd)
    piper.GripperCtrl(gripper_cmd, GRIP_EFFORT, 0x01, 0)

    elapsed = time.time() - start
    pct = 100 * (i+1) // len(trajectory)
    print(f"  [{i+1:3d}/{len(trajectory)}] ({pct:3d}%) {elapsed:5.1f}s  "
          f"J1={point[0]:6.1f}°  Grip={point[6]:5.1f}mm")

    time.sleep(TIME_PER_POINT)

total = time.time() - start
print(f"\nTrajectory complete in {total:.1f}s")
