#!/usr/bin/env python3
"""Run simple xArm 6 pick-and-place cycles from controller DI inputs.

This script connects to the xArm 6, moves to a safe start pose, opens the
gripper, and waits for controller DI0. When DI0 is triggered, the robot
keeps doing pick-and-place cycles in a loop. If DI1 is triggered, the
robot pauses between commands and waits for DI0 to resume.

Inputs:
    Robot connection through ROBOT_IP.
    Controller DI0 to start the repeated pick-and-place cycles.
    Controller DI0 to resume after pause.
    Controller DI1 to pause the process between robot commands.
    Pick/place Cartesian poses in millimeters and degrees.

Outputs:
    Robot motion and gripper commands sent to the xArm 6.

Important:
    DI1 is checked between robot commands. If the robot is already moving
    with wait=True, the current motion finishes before pause is handled by
    this simple script.
"""

import time

from xarm import version
from xarm.wrapper import XArmAPI


# User Settings
# TODO: Change this to the IP address shown on the xArm controller.
ROBOT_IP = "192.168.1.200"

# TODO: DI0 is 8. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI0_NUMBER = 8

# TODO: DI1 is 9. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI1_NUMBER = 9

# TODO: Set to 1 if your DI0 start input is high when pressed.
DI0_START_VALUE = 0

# TODO: Set to 1 if your DI1 pause input is high when pressed.
DI1_PAUSE_VALUE = 0

# TODO: Time between digital input reads, in seconds.
DIGITAL_READ_INTERVAL_SECONDS = 0.1

# TODO: Change after checking workspace. (q1, q2, q3, q4, q5, q6) deg.
SAFE_START_ANGLES = [3.2, -10.5, -90.5, 0.0, 101, -87.0]

# TODO: Change pick/place positions for your table/object.
# Format is (x, y, z, Rx, Ry, Rz) in mm and deg.
PICK_ABOVE_POSE  = [397, 24, 600.0, 180.0, 0.0, 90.0]
PICK_POSE        = [397, 24, 449, 180.0, 0.0, 90.0]
PLACE_ABOVE_POSE = [187, 320, 425.0, 180.0, 0.0, 90.0]
PLACE_POSE       = [187, 320, 425.0, 180.0, 0.0, 90.0]

# TODO: Change gripper values if your object needs different opening.
GRIPPER_OPEN_POSITION = 280
GRIPPER_CLOSED_POSITION = 0

# TODO: Cartesian speed range is 0.1 to 1000 mm/s.
TCP_SPEED = 1000

# TODO: Cartesian acceleration range is 1 to 50000 mm/s^2.
TCP_ACCELERATION = 10000

# TODO: Joint speed range is about 0.006 to 229 deg/s.
ANGLE_SPEED = 20

# TODO: Joint acceleration range is about 0.57 to 1146 deg/s^2.
ANGLE_ACCELERATION = 500

# TODO: Change if the gripper closes too slowly or too hard.
GRIPPER_SPEED = 10000

# TODO: Update if you know your real tool payload.
GRIPPER_MASS_KG = 0.82
GRIPPER_CENTER_OF_MASS_MM = [0, 0, 48]


def print_message(message):
    """Print a message with a timestamp."""
    current_time = time.strftime("%Y-%m-%d %H:%M:%S")
    print("[{}] {}".format(current_time, message))


def read_digital_input(arm, input_number, input_name):
    """Read one xArm controller digital input."""
    code, read_value = arm.get_cgpio_digital(input_number)

    if code != 0:
        print_message("{} read failed, code={}".format(input_name, code))
        return None

    return read_value


def input_is_triggered(arm, input_number, trigger_value, input_name):
    """Return True when a digital input equals its trigger value."""
    read_value = read_digital_input(arm, input_number, input_name)
    return read_value == trigger_value


def command_is_ok(code, command_name):
    """Return True when an xArm command succeeds."""
    if code == 0:
        return True

    print_message("{} failed, code={}".format(command_name, code))
    return False


def setup_robot(arm):
    """Prepare the robot for position control."""
    arm.clean_warn()
    arm.clean_error()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1)

    code = arm.set_counter_reset()
    if not command_is_ok(code, "set_counter_reset"):
        return False

    code = arm.set_servo_angle(
        angle=SAFE_START_ANGLES,
        speed=ANGLE_SPEED,
        mvacc=ANGLE_ACCELERATION,
        wait=True,
        radius=-1.0,
    )
    if not command_is_ok(code, "set_servo_angle"):
        return False

    code = arm.set_tcp_load(
        GRIPPER_MASS_KG,
        GRIPPER_CENTER_OF_MASS_MM,
    )
    if not command_is_ok(code, "set_tcp_load"):
        return False

    return open_gripper(arm)


def pause_if_needed(arm):
    """Pause on DI1, then wait for DI0 to resume."""
    if not input_is_triggered(
        arm,
        CONTROLLER_DI1_NUMBER,
        DI1_PAUSE_VALUE,
        "DI1",
    ):
        return True

    print_message("DI1 pause triggered.")
    print_message("Release DI1, then press DI0 to resume.")

    while True:
        if not input_is_triggered(
            arm,
            CONTROLLER_DI1_NUMBER,
            DI1_PAUSE_VALUE,
            "DI1",
        ):
            print_message("DI1 released.")
            return wait_for_di0_start(arm)

        time.sleep(DIGITAL_READ_INTERVAL_SECONDS)


def wait_for_di0_start(arm):
    """Wait until DI0 is triggered."""
    print_message("Waiting for DI0 start trigger.")

    while True:
        if input_is_triggered(
            arm,
            CONTROLLER_DI0_NUMBER,
            DI0_START_VALUE,
            "DI0",
        ):
            print_message("DI0 start triggered.")
            return wait_for_di0_release(arm)

        time.sleep(DIGITAL_READ_INTERVAL_SECONDS)


def wait_for_di0_release(arm):
    """Wait until DI0 is released after the start trigger."""
    print_message("Waiting for DI0 release.")

    while True:
        if not input_is_triggered(
            arm,
            CONTROLLER_DI0_NUMBER,
            DI0_START_VALUE,
            "DI0",
        ):
            print_message("DI0 released.")
            return True

        time.sleep(DIGITAL_READ_INTERVAL_SECONDS)


def move_to_pose(arm, pose, command_name):
    """Move the tool center point to one Cartesian pose."""
    pause_if_needed(arm)

    code = arm.set_position(
        *pose,
        speed=TCP_SPEED,
        mvacc=TCP_ACCELERATION,
        radius=0.0,
        wait=True,
    )
    if not command_is_ok(code, command_name):
        return False

    pause_if_needed(arm)
    return True


def open_gripper(arm):
    """Open the xArm gripper."""
    pause_if_needed(arm)

    code = arm.set_gripper_position(
        GRIPPER_OPEN_POSITION,
        wait=True,
        speed=GRIPPER_SPEED,
        auto_enable=True,
    )
    return command_is_ok(code, "open_gripper")


def close_gripper(arm):
    """Close the xArm gripper."""
    pause_if_needed(arm)

    code = arm.set_gripper_position(
        GRIPPER_CLOSED_POSITION,
        wait=True,
        speed=GRIPPER_SPEED,
        auto_enable=True,
    )
    return command_is_ok(code, "close_gripper")


def pick_and_place_cycle(arm):
    """Run one pick-and-place cycle."""
    if not move_to_pose(arm, PICK_ABOVE_POSE, "move_pick_above"):
        return False
    if not move_to_pose(arm, PICK_POSE, "move_pick"):
        return False

    if not close_gripper(arm):
        return False

    if not move_to_pose(arm, PICK_ABOVE_POSE, "move_up_from_pick"):
        return False
    if not move_to_pose(arm, PLACE_ABOVE_POSE, "move_place_above"):
        return False
    if not move_to_pose(arm, PLACE_POSE, "move_place"):
        return False

    if not open_gripper(arm):
        return False

    if not move_to_pose(arm, PLACE_ABOVE_POSE, "move_up_from_place"):
        return False

    return move_to_pose(arm, PICK_ABOVE_POSE, "return_pick_above")


def run_pick_and_place_loop(arm):
    """Wait for DI0 once, then repeat pick-and-place."""
    if not setup_robot(arm):
        return

    if not wait_for_di0_start(arm):
        return

    print_message("Starting repeated pick-and-place cycles.")

    while True:
        if not pick_and_place_cycle(arm):
            print_message("Pick-and-place loop stopped.")
            return

        print_message("Cycle complete. Starting next cycle.")


if __name__ == "__main__":
    print_message("xArm-Python-SDK Version:{}".format(version.__version__))

    read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
    time.sleep(0.5)

    try:
        run_pick_and_place_loop(read_write_arm)
    finally:
        read_write_arm.disconnect()
