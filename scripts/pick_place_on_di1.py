#!/usr/bin/env python3
"""Run one xArm 6 pick-and-place cycle when controller DI1 is triggered.

This script connects to the xArm, moves to a safe start pose, waits for
controller DI1, then performs one pick-and-place cycle.

Inputs:
    Robot connection through ROBOT_IP.
    Controller digital input DI1.
    Pick/place Cartesian poses in millimeters and degrees.

Outputs:
    Robot motion commands sent to the xArm 6 after DI1 is triggered.
"""

import os
import select
import sys
import termios
import time
import traceback
import tty

from xarm import version
from xarm.wrapper import XArmAPI


# User Settings
# TODO: Change this to the IP address shown on the xArm controller.
ROBOT_IP = "192.168.1.200"

# TODO: DI1 is 9. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI1_NUMBER = 9

# TODO: Set to 1 if your sensor/input is high when triggered.
DI1_TRIGGER_VALUE = 0

# TODO: Time between DI1 reads, in seconds.
DI1_READ_INTERVAL_SECONDS = 0.1

# TODO: Launcher sets this to skip waiting after DI1 already triggered.
RUN_WITHOUT_DI1_WAIT = os.environ.get("RUN_WITHOUT_DI1_WAIT") == "1"

# TODO: Change after checking workspace. (q1, q2, q3, q4, q5, q6) deg
SAFE_START_ANGLES = [3.2, -10.5, -90.5, 0.0, 101, -87.0]

# TODO: Change pick/place positions for your table/object.
# Format is (x,y,z,Rx,Ry,Rz) in mm and deg.
PICK_ABOVE_POSE  = [397, 24, 600.0, 180.0, 0.0, 90.0]
PICK_POSE        = [397, 24, 449, 180.0, 0.0, 90.0]
PLACE_ABOVE_POSE = [300, 24, 520.0, 180.0, 0.0, 90.0]
PLACE_POSE       = [300, 24, 373, 180.0, 0.0, 90.0]

# PICK_ABOVE_POSE  = [397, 24, 600.0, 180.0, 0.0, 90.0]
# PICK_POSE        = [397, 24, 600.0, 180.0, 0.0, 90.0]
# PLACE_ABOVE_POSE = [397, 24, 600.0, 180.0, 0.0, 90.0]
# PLACE_POSE       = [397, 24, 600.0, 180.0, 0.0, 90.0]

# TODO: Change gripper values if your object needs different opening.
GRIPPER_OPEN_POSITION = 280
GRIPPER_CLOSED_POSITION = 0

TCP_SPEED = 200
TCP_ACCELERATION = 5000
ANGLE_SPEED = 20
ANGLE_ACCELERATION = 500
GRIPPER_SPEED = 5000

# xArm gripper payload only. Update if you know your real tool payload.
GRIPPER_MASS_KG = 0.82
GRIPPER_CENTER_OF_MASS_MM = [0, 0, 48]


def read_keyboard_key():
    """Read one keyboard key without blocking the DI1 loop."""
    read_ready, _, _ = select.select([sys.stdin], [], [], 0)

    if not read_ready:
        return ""

    return sys.stdin.read(1)


class PickPlaceOnDi1Robot:
    """Small readable wrapper for DI1-triggered pick-and-place."""

    def __init__(self, arm):
        self.alive = True
        self.arm = arm
        self.robot_init()

    def robot_init(self):
        """Prepare the robot for position control."""
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)
        self.arm.register_error_warn_changed_callback(
            self.error_warn_changed_callback
        )
        self.arm.register_state_changed_callback(
            self.state_changed_callback
        )

    def error_warn_changed_callback(self, data):
        """Stop the script if the robot reports an error."""
        if data and data["error_code"] != 0:
            self.alive = False
            self.print_message("error={}, quit".format(data["error_code"]))
            self.arm.release_error_warn_changed_callback(
                self.error_warn_changed_callback
            )

    def state_changed_callback(self, data):
        """Stop the script if the robot enters state 4."""
        if data and data["state"] == 4:
            self.alive = False
            self.print_message("state=4, quit")
            self.arm.release_state_changed_callback(
                self.state_changed_callback
            )

    def check_code(self, code, label):
        """Check return code after each xArm command."""
        if not self.is_alive or code != 0:
            self.alive = False
            state_result = self.arm.get_state()
            error_result = self.arm.get_err_warn_code()
            self.print_message(
                "{}, code={}, connected={}, state={}, error={}, "
                "state_result={}, error_result={}".format(
                    label,
                    code,
                    self.arm.connected,
                    self.arm.state,
                    self.arm.error_code,
                    state_result,
                    error_result,
                )
            )
        return self.is_alive

    @staticmethod
    def print_message(*args):
        """Print a timestamped message with the current line number."""
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print(
                "[{}][{}] {}".format(
                    time.strftime(
                        "%Y-%m-%d %H:%M:%S",
                        time.localtime(time.time()),
                    ),
                    stack_tuple[1],
                    " ".join(map(str, args)),
                )
            )
        except Exception:
            print(*args)

    @property
    def is_alive(self):
        """Return True when the robot connection is usable."""
        if self.alive and self.arm.connected and self.arm.error_code == 0:
            if self.arm.state == 5:
                wait_count = 0
                while self.arm.state == 5 and wait_count < 5:
                    wait_count += 1
                    time.sleep(0.1)
            return self.arm.state < 4
        return False

    def wait_for_di1_trigger(self):
        """Wait until controller DI1 reaches DI1_TRIGGER_VALUE."""
        self.print_message("Waiting for DI1 trigger...")
        self.print_message("Press q to quit.")

        while self.is_alive:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Quitting.")
                return False

            code, read_di1_value = self.arm.get_cgpio_digital(
                CONTROLLER_DI1_NUMBER
            )
            if not self.check_code(code, "get_cgpio_digital_DI1"):
                return False

            self.print_message("DI1={}".format(read_di1_value))

            if read_di1_value == DI1_TRIGGER_VALUE:
                self.print_message("DI1 triggered.")
                return True

            time.sleep(DI1_READ_INTERVAL_SECONDS)

        return False

    def wait_for_di1_release(self):
        """Wait until controller DI1 is no longer triggered."""
        self.print_message("Waiting for DI1 release...")

        while self.is_alive:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Quitting.")
                return False

            code, read_di1_value = self.arm.get_cgpio_digital(
                CONTROLLER_DI1_NUMBER
            )
            if not self.check_code(code, "get_cgpio_digital_DI1"):
                return False

            if read_di1_value != DI1_TRIGGER_VALUE:
                self.print_message("DI1 released.")
                return True

            time.sleep(DI1_READ_INTERVAL_SECONDS)

        return False

    def set_gripper_open(self):
        """Open the xArm gripper."""
        code = self.arm.set_gripper_position(
            GRIPPER_OPEN_POSITION,
            wait=True,
            speed=GRIPPER_SPEED,
            auto_enable=True,
        )
        return self.check_code(code, "set_gripper_open")

    def set_gripper_closed(self):
        """Close the xArm gripper."""
        code = self.arm.set_gripper_position(
            GRIPPER_CLOSED_POSITION,
            wait=True,
            speed=GRIPPER_SPEED,
            auto_enable=True,
        )
        return self.check_code(code, "set_gripper_closed")

    def move_to_pose(self, pose, label):
        """Move the tool center point to one Cartesian pose."""
        code = self.arm.set_position(
            *pose,
            speed=TCP_SPEED,
            mvacc=TCP_ACCELERATION,
            radius=0.0,
            wait=True,
        )
        return self.check_code(code, label)

    def pick_and_place_cycle(self):
        """Run one pick-and-place cycle."""
        if not self.move_to_pose(PICK_ABOVE_POSE, "move_pick_above"):
            return False
        if not self.move_to_pose(PICK_POSE, "move_pick"):
            return False

        if not self.set_gripper_closed():
            return False

        if not self.move_to_pose(PICK_ABOVE_POSE, "move_up_from_pick"):
            return False
        if not self.move_to_pose(PLACE_ABOVE_POSE, "move_place_above"):
            return False
        if not self.move_to_pose(PLACE_POSE, "move_place"):
            return False

        if not self.set_gripper_open():
            return False

        return self.move_to_pose(PLACE_ABOVE_POSE, "move_up_from_place")

    def run(self):
        """Run pick-and-place each time DI1 is triggered."""
        try:
            code = self.arm.set_counter_reset()
            if not self.check_code(code, "set_counter_reset"):
                return

            code = self.arm.set_servo_angle(
                angle=SAFE_START_ANGLES,
                speed=ANGLE_SPEED,
                mvacc=ANGLE_ACCELERATION,
                wait=True,
                radius=-1.0,
            )
            if not self.check_code(code, "set_servo_angle"):
                return

            if not self.set_gripper_open():
                return

            code = self.arm.set_tcp_load(
                GRIPPER_MASS_KG,
                GRIPPER_CENTER_OF_MASS_MM,
            )
            if not self.check_code(code, "set_tcp_load_gripper"):
                return

            if RUN_WITHOUT_DI1_WAIT:
                self.pick_and_place_cycle()
                return

            while self.is_alive:
                if not self.wait_for_di1_trigger():
                    return

                if not self.pick_and_place_cycle():
                    return

                if not self.wait_for_di1_release():
                    return

        except Exception as error:
            self.print_message("MainException: {}".format(error))
        finally:
            self.alive = False
            self.arm.release_error_warn_changed_callback(
                self.error_warn_changed_callback
            )
            self.arm.release_state_changed_callback(
                self.state_changed_callback
            )


if __name__ == "__main__":
    PickPlaceOnDi1Robot.print_message(
        "xArm-Python-SDK Version:{}".format(version.__version__)
    )

    old_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
        time.sleep(0.5)

        robot = PickPlaceOnDi1Robot(read_write_arm)
        robot.run()

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings,
        )
