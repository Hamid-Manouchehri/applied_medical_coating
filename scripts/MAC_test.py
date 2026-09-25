#!/usr/bin/env python3
"""Modular xArm 6 test script: joint-space via points + gripper moves.

This script gives you two building blocks:

    move_to_via(via_angles, label)
        Moves the robot to a joint-space via point (q1..q6, degrees).

    set_gripper(position, label)
        Opens/closes the gripper to any position you choose.

You define your own named via points and gripper positions in the
"User Settings" section below, then call move_to_via() / set_gripper()
in any order you want inside run_sequence(). Nothing here forces a
fixed pick-and-place shape.

Inputs:
    Robot connection through ROBOT_IP.
    Joint-space via points in degrees.
    Gripper positions (device units, 0 = closed).
    Keyboard key "q", pressed at any time, to stop the script.

Outputs:
    Robot motion and gripper commands sent to the xArm 6.

Important:
    Pressing q calls set_state(4), which can interrupt a move that is
    already in progress. The script then stops cleanly instead of
    continuing to the next action.
"""

import select
import sys
import termios
import threading
import time
import traceback
import tty

from xarm import version
from xarm.wrapper import XArmAPI


# User Settings
# TODO: Change this to the IP address shown on the xArm controller.
ROBOT_IP = "192.168.1.200"

# TODO: Joint-space via points, (q1, q2, q3, q4, q5, q6) in degrees.
# Add, remove, or rename these freely, then use them in run_sequence().
VIA_HOME = [3.2, -10.5, -90.5, 0.0, 101, -87.0]
VIA_POINT_1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
VIA_POINT_2 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# TODO: Gripper positions, device units. Typically 0 (closed) up to
# about 850 (fully open) for the xArm gripper. Use any value you want.
GRIPPER_OPEN_POSITION = 280
GRIPPER_CLOSED_POSITION = 0

# TODO: Joint speed range is about 0.006 to 229 deg/s.
ANGLE_SPEED = 20

# TODO: Joint acceleration range is about 0.57 to 1146 deg/s^2.
ANGLE_ACCELERATION = 500

# TODO: Change if the gripper closes too slowly or too hard.
GRIPPER_SPEED = 10000

# TODO: Update if you know your real tool payload.
GRIPPER_MASS_KG = 0.82
GRIPPER_CENTER_OF_MASS_MM = [0, 0, 48]

# TODO: Keyboard poll interval, in seconds. Smaller reacts to q
# faster, larger uses less CPU.
KEYBOARD_READ_INTERVAL_SECONDS = 0.05


def read_keyboard_key():
    """Read one keyboard key without blocking, or "" if none waiting."""
    read_ready, _, _ = select.select([sys.stdin], [], [], 0)

    if not read_ready:
        return ""

    return sys.stdin.read(1)


class ModularRobot:
    """Via-point and gripper actions you call in any order. Press q to
    stop."""

    def __init__(self, arm):
        self.alive = True
        self.arm = arm
        self.quit_requested = False
        self.keyboard_monitor_running = False
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
            if self.quit_requested:
                self.print_message("state=4 from q stop.")
                return

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

    # --- Keyboard "q to stop" ---

    def start_keyboard_monitor(self):
        """Start the background thread that watches for q."""
        self.keyboard_monitor_running = True
        monitor_thread = threading.Thread(
            target=self.keyboard_monitor_loop,
            daemon=True,
        )
        monitor_thread.start()

    def keyboard_monitor_loop(self):
        """Stop the robot the moment q is pressed, even mid-move."""
        while self.keyboard_monitor_running and self.arm.connected:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Stopping.")
                self.quit_requested = True
                self.keyboard_monitor_running = False
                self.arm.set_state(4)
                return

            time.sleep(KEYBOARD_READ_INTERVAL_SECONDS)

    # --- Building blocks: call these in any order in run_sequence() ---

    def move_to_via(self, via_angles, label):
        """Move the robot to one joint-space via point."""
        if self.quit_requested:
            return False

        code = self.arm.set_servo_angle(
            angle=via_angles,
            speed=ANGLE_SPEED,
            mvacc=ANGLE_ACCELERATION,
            wait=True,
            radius=-1.0,
        )
        return self.check_code(code, label) and not self.quit_requested

    def set_gripper(self, position, label):
        """Move the gripper to any position you choose."""
        if self.quit_requested:
            return False

        code = self.arm.set_gripper_position(
            position,
            wait=True,
            speed=GRIPPER_SPEED,
            auto_enable=True,
        )
        return self.check_code(code, label) and not self.quit_requested

    # --- Your sequence ---

    def run_sequence(self):
        """Call move_to_via() / set_gripper() here in any order you
        want. This example just demonstrates the pattern."""
        if not self.move_to_via(VIA_HOME, "via_home"):
            return
        if not self.set_gripper(GRIPPER_OPEN_POSITION, "gripper_open"):
            return
        if not self.move_to_via(VIA_POINT_1, "via_point_1"):
            return
        if not self.set_gripper(GRIPPER_CLOSED_POSITION, "gripper_close"):
            return
        if not self.move_to_via(VIA_POINT_2, "via_point_2"):
            return
        if not self.set_gripper(GRIPPER_OPEN_POSITION, "gripper_open"):
            return
        self.move_to_via(VIA_HOME, "via_home")

    def run(self):
        """Set up the robot, watch for q, then run your sequence."""
        try:
            code = self.arm.set_tcp_load(
                GRIPPER_MASS_KG,
                GRIPPER_CENTER_OF_MASS_MM,
            )
            if not self.check_code(code, "set_tcp_load_gripper"):
                return

            self.start_keyboard_monitor()
            self.print_message("Running. Press q to stop.")

            self.run_sequence()

            if self.quit_requested:
                self.print_message("Stopped by q.")
            else:
                self.print_message("Sequence complete.")

        except Exception as error:
            self.print_message("MainException: {}".format(error))
        finally:
            self.keyboard_monitor_running = False
            self.alive = False
            self.arm.release_error_warn_changed_callback(
                self.error_warn_changed_callback
            )
            self.arm.release_state_changed_callback(
                self.state_changed_callback
            )


if __name__ == "__main__":
    ModularRobot.print_message(
        "xArm-Python-SDK Version:{}".format(version.__version__)
    )

    old_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
        time.sleep(0.5)

        robot = ModularRobot(read_write_arm)
        robot.run()

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings,
        )
        read_write_arm.disconnect()
