#!/usr/bin/env python3
# waypoint_sequence.py
# -------------------------------------------------------
# Records waypoints by jogging the arm, then replays them.
#
# This example shows the manual waypoint workflow without
# the full interactive UI of piper_arm_hardware.py.
#
# Usage:
#   1. Jog the arm to a position (edit the joint values below)
#   2. Add it to WAYPOINTS
#   3. Run the script — arm moves through all waypoints
# -------------------------------------------------------

import time
from piper_sdk import C_PiperInterface_V2

# ----------------------------------------------------------
# EDIT THIS SECTION — Define your waypoints
#
# Each waypoint: [J1, J2, J3, J4, J5, J6, Gripper_mm]
# Joints in DEGREES, Gripper in mm
#
# To find good values: run piper_arm_hardware.py,
# jog the arm to the position you want, press P and W
# to record the joint angles, then copy them here.
# ----------------------------------------------------------
WAYPOINTS = [
    [  0,   0,   0,   0,   0,   0,   0],   # Home
    [ 10,  10,   0,   0,   0,   0,   0],   # Position 1
    [ 20,  20,  10,   0,   0,   0,  20],   # Position 2 (gripper open)
    [ 10,  30,  10,   0,   0,   0,  40],   # Position 3
    [  0,  20,   0,   0,   0,   0,   0],   # Position 4 (gripper closed)
    [  0,   0,   0,   0,   0,   0,   0],   # Back to home
]

DWELL_TIME   = 2.0     # Seconds to wait at each waypoint
GRIP_EFFORT  = 500     # Gripper force (0-1000)
SPEED        = 3       # Motion speed (1-10), use 3 for safety

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
print(f"Speed set to {SPEED}/10\n")

# ----------------------------------------------------------
# Execute waypoint sequence
# ----------------------------------------------------------
print(f"Executing {len(WAYPOINTS)} waypoints...")
print(f"Dwell time per waypoint: {DWELL_TIME}s\n")

for i, wp in enumerate(WAYPOINTS):
    print(f"--- Waypoint {i+1}/{len(WAYPOINTS)} ---")
    print(f"  Joints:  {wp[:6]}")
    print(f"  Gripper: {wp[6]:.1f}mm")

    # Convert degrees → millidegrees for SDK
    joints_cmd  = [int(j * 1000) for j in wp[:6]]
    gripper_cmd = int(wp[6] * 1000)

    # Send commands
    piper.JointCtrl(*joints_cmd)
    piper.GripperCtrl(gripper_cmd, GRIP_EFFORT, 0x01, 0)

    # Wait for motion to complete
    time.sleep(DWELL_TIME)

    # Read back actual position
    msg = piper.GetArmJointMsgs()
    if msg:
        js = msg.joint_state
        actual = [js.joint_1/1000, js.joint_2/1000, js.joint_3/1000,
                  js.joint_4/1000, js.joint_5/1000, js.joint_6/1000]
        print(f"  Actual:  {[f'{j:.1f}' for j in actual]}")

    print()

print("Waypoint sequence complete!")
print("Arm is at the last waypoint (home if you kept the default list).")
