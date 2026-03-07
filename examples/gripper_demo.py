#!/usr/bin/env python3
# gripper_demo.py
# -------------------------------------------------------
# Demonstrates gripper open and close on the PIPER arm.
#
# The gripper is controlled with GripperCtrl():
#   GripperCtrl(angle, effort, ctrl_mode, spare)
#
#   angle     — target angle in millidegrees
#               0       = fully closed
#               80000   = fully open (~80mm)
#   effort    — gripping force (0–1000), 500 is a safe default
#   ctrl_mode — 0x01 = position control (what we always use)
#   spare     — always 0
# -------------------------------------------------------

import time
from piper_sdk import C_PiperInterface_V2

# ----------------------------------------------------------
# Constants
# ----------------------------------------------------------
GRIPPER_CLOSED  = 0         # millidegrees
GRIPPER_OPEN    = 80000     # millidegrees  (~80mm)
GRIPPER_HALF    = 40000     # millidegrees  (~40mm)
GRIP_EFFORT     = 500       # Moderate gripping force
CTRL_MODE       = 0x01      # Position control

# ----------------------------------------------------------
# Connect
# ----------------------------------------------------------
piper = C_PiperInterface_V2("can0",
                             start_sdk_joint_limit=True,
                             start_sdk_gripper_limit=True)
piper.ConnectPort()

print("Connecting...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Connected!")

piper.MotionCtrl_2(0x01, 0x01, 3, 0x00)   # Speed 3/10

# ----------------------------------------------------------
# Helper: read and print current gripper position
# ----------------------------------------------------------
def read_gripper():
    msg = piper.GetArmGripperMsgs()
    if msg:
        angle = msg.gripper_state.grippers_angle
        print(f"  Gripper position: {angle/1000:.1f} mm")
        return angle
    print("  Could not read gripper")
    return None

# ----------------------------------------------------------
# Demo sequence
# ----------------------------------------------------------
print("\n--- Gripper Demo ---\n")

print("1. Opening gripper fully (80mm)...")
piper.GripperCtrl(GRIPPER_OPEN, GRIP_EFFORT, CTRL_MODE, 0)
time.sleep(2)
read_gripper()

print("\n2. Closing gripper to half (40mm)...")
piper.GripperCtrl(GRIPPER_HALF, GRIP_EFFORT, CTRL_MODE, 0)
time.sleep(2)
read_gripper()

print("\n3. Closing gripper fully (0mm)...")
piper.GripperCtrl(GRIPPER_CLOSED, GRIP_EFFORT, CTRL_MODE, 0)
time.sleep(2)
read_gripper()

print("\n4. Opening gripper to pick position (30mm)...")
piper.GripperCtrl(30000, GRIP_EFFORT, CTRL_MODE, 0)
time.sleep(2)
read_gripper()

print("\n5. Simulating grasp — closing slowly...")
for mm in range(30, -1, -5):
    piper.GripperCtrl(mm * 1000, GRIP_EFFORT, CTRL_MODE, 0)
    time.sleep(0.5)
read_gripper()

print("\n6. Release — opening fully...")
piper.GripperCtrl(GRIPPER_OPEN, GRIP_EFFORT, CTRL_MODE, 0)
time.sleep(2)
read_gripper()

print("\nGripper demo complete!")
