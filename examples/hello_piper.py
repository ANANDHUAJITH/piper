#!/usr/bin/env python3
# hello_piper.py
# -------------------------------------------------------
# Minimal "Hello World" example for the PIPER robotic arm.
# Moves Joint 1 forward 10°, waits, then returns to home.
#
# Requires: piper_sdk, CAN bus active (hardware) or
#           RViz + MoveIt running (simulation)
# -------------------------------------------------------

import time
from piper_sdk import C_PiperInterface_V2

# ----------------------------------------------------------
# 1. Connect to robot
# ----------------------------------------------------------
piper = C_PiperInterface_V2(
    "can0",
    start_sdk_joint_limit=True,    # Enforce joint limits in software
    start_sdk_gripper_limit=True
)
piper.ConnectPort()

print("Connecting to PIPER arm...")
while not piper.EnablePiper():
    time.sleep(0.01)
print("Connected!")

# ----------------------------------------------------------
# 2. Set a safe slow speed (1 = slowest, 10 = fastest)
# ----------------------------------------------------------
piper.MotionCtrl_2(0x01, 0x01, 3, 0x00)   # Speed 3/10
print("Speed set to 3/10 (slow)")

# ----------------------------------------------------------
# 3. Read initial joint positions
#
#    GetArmJointMsgs() returns a message object.
#    joint_state contains .joint_1 through .joint_6
#    Units: millidegrees (1000 = 1 degree)
# ----------------------------------------------------------
msg = piper.GetArmJointMsgs()
js = msg.joint_state
joints = [js.joint_1, js.joint_2, js.joint_3,
          js.joint_4, js.joint_5, js.joint_6]

print(f"Starting joints (degrees): {[j/1000 for j in joints]}")

# ----------------------------------------------------------
# 4. Move Joint 1 forward 10 degrees
#
#    JointCtrl takes 6 joint values in millidegrees.
#    We only change joints[0] (Joint 1 = base rotation).
# ----------------------------------------------------------
print("\nMoving Joint 1 forward 10°...")
joints[0] += 10 * 1000   # Add 10 degrees (in millidegrees)
piper.JointCtrl(*joints)  # Send command to all 6 joints
time.sleep(2)             # Wait 2 seconds for motion to complete

# ----------------------------------------------------------
# 5. Print new end-effector position
#
#    GetArmEndPoseMsgs() returns the Cartesian position.
#    Units for X/Y/Z: units need dividing by 1000 → mm
#    Units for RX/RY/RZ: units need dividing by 1000 → degrees
# ----------------------------------------------------------
pose = piper.GetArmEndPoseMsgs()
if pose:
    p = pose.end_pose
    print(f"\nEnd-effector position after move:")
    print(f"  X  = {p.X_axis/1000:.2f} mm")
    print(f"  Y  = {p.Y_axis/1000:.2f} mm")
    print(f"  Z  = {p.Z_axis/1000:.2f} mm")
    print(f"  RX = {p.RX_axis/1000:.2f}°")
    print(f"  RY = {p.RY_axis/1000:.2f}°")
    print(f"  RZ = {p.RZ_axis/1000:.2f}°")

# ----------------------------------------------------------
# 6. Return all joints to HOME (all zeros)
#
#    The home position has all joint angles = 0.
#    This is a safe resting configuration.
# ----------------------------------------------------------
print("\nReturning to home position...")
home = [0, 0, 0, 0, 0, 0]
piper.JointCtrl(*home)
time.sleep(2)

print("\nDone! PIPER arm is back at home.")
