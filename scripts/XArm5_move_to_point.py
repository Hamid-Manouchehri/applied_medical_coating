#!/usr/bin/env python3
"""Move the xArm 5 to one point with MoveJ or MoveL, to check a via point.

Set MOVE_TYPE below, run the script, and the robot moves once to the
chosen point. Afterwards it prints where it ended up in both joint
space and Cartesian space, so you can compare with what you expected.

    MoveJ: joint-space move to TARGET_ANGLES (q1..q5, degrees).
    MoveL: straight-line move to TARGET_POSE
           ([x, y, z, roll, pitch, yaw], mm and degrees).

Inputs:
    Robot connection through ROBOT_IP.
    MOVE_TYPE, TARGET_ANGLES or TARGET_POSE.

Outputs:
    One robot move, then the final joint angles and Cartesian pose
    printed in the terminal.

Important:
    Keep the emergency stop within reach. The move goes straight from
    the current pose to the target, so check that the path is clear.
    Start with a low speed.
"""

import time

from xarm.wrapper import XArmAPI


# User Settings
# TODO: Change this to the IP address shown on the xArm controller.
ROBOT_IP = "192.168.1.229"

# TODO: "J" for MoveJ (joint space), "L" for MoveL (straight line).
MOVE_TYPE = "J"

# TODO: MoveJ target, (q1, q2, q3, q4, q5) in degrees.
TARGET_ANGLES = [1.3, -20.7, -15.2, 35.8, 91.3]

# TODO: MoveL target, [x, y, z, roll, pitch, yaw] in mm and degrees.
# On the xArm 5, roll must be 180 and pitch 0.
TARGET_POSE = [287.4, 7.4, 157.1, 180.0, 0.0, -90.0]

# TODO: MoveJ joint speed, 0.006 to 229 deg/s. Keep it low for tests.
ANGLE_SPEED = 20

# TODO: MoveJ joint acceleration, 0.57 to 1146 deg/s^2.
ANGLE_ACCELERATION = 100

# TODO: MoveL tool speed, 0.1 to 1000 mm/s. Keep it low for tests.
LINEAR_SPEED = 50

# TODO: MoveL tool acceleration, 1 to 50000 mm/s^2.
LINEAR_ACCELERATION = 500


def move_j(arm, target_angles):
    """MoveJ: joint-space move to target_angles (degrees)."""
    code = arm.set_servo_angle(
        angle=target_angles,
        speed=ANGLE_SPEED,
        mvacc=ANGLE_ACCELERATION,
        wait=True,
        radius=-1.0,
    )
    print("MoveJ finished, code =", code)


def move_l(arm, target_pose):
    """MoveL: straight-line move to target_pose (mm and degrees)."""
    code = arm.set_position(
        *target_pose,
        speed=LINEAR_SPEED,
        mvacc=LINEAR_ACCELERATION,
        radius=-1.0,
        wait=True,
    )
    print("MoveL finished, code =", code)


if __name__ == "__main__":
    arm = XArmAPI(ROBOT_IP, baud_checkset=False)
    time.sleep(0.5)

    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1)

    if MOVE_TYPE == "J":
        print("MoveJ to angles:", TARGET_ANGLES)
        move_j(arm, TARGET_ANGLES)
    else:
        print("MoveL to pose:", TARGET_POSE)
        move_l(arm, TARGET_POSE)

    _, final_angles = arm.get_servo_angle(is_radian=False)
    _, final_pose = arm.get_position(is_radian=False)
    final_angles = [round(a, 1) for a in final_angles[:5]]
    final_pose = [round(p, 1) for p in final_pose]
    print("Final joint angles (deg):", final_angles)
    print("Final pose (mm, deg):    ", final_pose)

    arm.disconnect()
