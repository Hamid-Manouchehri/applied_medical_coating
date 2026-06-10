#!/usr/bin/env python3
"""Run xArm 6 pick-and-place cycles from DI0, with DI1 stop and DI2 home.

This script connects to the xArm, moves to a safe start pose, waits for
controller DI0, then performs one pick-and-place cycle. During motion,
controller DI1 is monitored as an immediate stop input. After a DI1 stop,
controller DI2 opens the gripper and moves the robot to PICK_ABOVE_POSE.

Inputs:
    Robot connection through ROBOT_IP.
    Controller digital input DI0 for cycle start.
    Controller digital input DI1 for immediate stop.
    Controller digital input DI2 to return to PICK_ABOVE_POSE.
    Pick/place Cartesian poses in millimeters and degrees.

Outputs:
    Robot motion commands sent to the xArm 6 after DI0 is triggered.
"""

import os
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

# TODO: DI0 is 8. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI0_NUMBER = 8

# TODO: DI1 is 9. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI1_NUMBER = 9

# TODO: DI2 is 10. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI2_NUMBER = 10

# TODO: Set to 1 if your DI0 start input is high when pressed.
DI0_START_VALUE = 0

# TODO: Set to 1 if your DI1 stop input is high when pressed.
DI1_STOP_VALUE = 0

# TODO: Set to 1 if your DI2 restart input is high when pressed.
DI2_RESTART_VALUE = 0

# TODO: Time between digital input reads, in seconds.
DIGITAL_READ_INTERVAL_SECONDS = 0.1

# TODO: Launcher sets this to skip waiting after DI0 already triggered.
RUN_WITHOUT_DI0_WAIT = os.environ.get("RUN_WITHOUT_DI0_WAIT") == "1"

# TODO: Change after checking workspace. (q1, q2, q3, q4, q5, q6) deg
SAFE_START_ANGLES = [3.2, -10.5, -90.5, 0.0, 101, -87.0]

# TODO: Change pick/place positions for your table/object.
# Format is (x,y,z,Rx,Ry,Rz) in mm and deg.
PICK_ABOVE_POSE  = [397, 24, 600.0, 180.0, 0.0, 90.0]
PICK_POSE        = [397, 24, 449, 180.0, 0.0, 90.0]
PLACE_ABOVE_POSE = [187, 320, 425.0, 180.0, 0.0, 90.0]
PLACE_POSE       = [187, 320, 425.0, 180.0, 0.0, 90.0]

# TODO: Change gripper values if your object needs different opening.
GRIPPER_OPEN_POSITION = 280
GRIPPER_CLOSED_POSITION = 0

TCP_SPEED = 1000
TCP_ACCELERATION = 10000
ANGLE_SPEED = 20
ANGLE_ACCELERATION = 500
GRIPPER_SPEED = 10000

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
    """Small wrapper for DI0 start, DI1 stop, and DI2 home."""

    def __init__(self, arm):
        self.alive = True
        self.arm = arm
        self.di1_stop_monitor_active = False
        self.di1_stop_triggered = False
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
            if self.di1_stop_triggered:
                self.print_message("state=4 from DI1 stop.")
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

    def wait_for_di0_start(self):
        """Wait until controller DI0 reaches DI0_START_VALUE."""
        self.print_message("Waiting for DI0 start trigger...")
        self.print_message("Press q to quit.")

        while self.is_alive:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Quitting.")
                return False

            code, read_di0_value = self.arm.get_cgpio_digital(
                CONTROLLER_DI0_NUMBER
            )
            if not self.check_code(code, "get_cgpio_digital_DI0"):
                return False

            self.print_message("DI0={}".format(read_di0_value))

            if read_di0_value == DI0_START_VALUE:
                self.print_message("DI0 start triggered.")
                return True

            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)

        return False

    def wait_for_di0_release(self):
        """Wait until controller DI0 is no longer triggered."""
        self.print_message("Waiting for DI0 release...")

        while self.is_alive:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Quitting.")
                return False

            code, read_di0_value = self.arm.get_cgpio_digital(
                CONTROLLER_DI0_NUMBER
            )
            if not self.check_code(code, "get_cgpio_digital_DI0"):
                return False

            if read_di0_value != DI0_START_VALUE:
                self.print_message("DI0 released.")
                return True

            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)

        return False

    def wait_for_di2_home(self):
        """Wait until controller DI2 reaches DI2_RESTART_VALUE."""
        self.print_message("Waiting for DI2 home trigger...")
        self.print_message("Press q to quit.")

        while self.arm.connected:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Quitting.")
                return False

            code, read_di2_value = self.arm.get_cgpio_digital(
                CONTROLLER_DI2_NUMBER
            )
            if code != 0:
                self.print_message(
                    "get_cgpio_digital_DI2 failed, code={}".format(code)
                )
                time.sleep(DIGITAL_READ_INTERVAL_SECONDS)
                continue

            self.print_message("DI2={}".format(read_di2_value))

            if read_di2_value == DI2_RESTART_VALUE:
                self.print_message("DI2 home triggered.")
                return True

            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)

        return False

    def go_home_after_di1_stop(self):
        """Open gripper and move to PICK_ABOVE_POSE after DI1 stop."""
        if not self.wait_for_di2_home():
            return False

        self.print_message("Restarting robot motion state...")
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)

        self.alive = True
        self.di1_stop_triggered = False

        if not self.set_gripper_open():
            return False

        if not self.move_to_pose(PICK_ABOVE_POSE, "di2_return_pick_above"):
            return False

        self.print_message("DI2 home complete. Waiting for DI0.")
        return True

    def start_di1_stop_monitor(self):
        """Start background DI1 monitor during robot motion."""
        self.di1_stop_triggered = False
        self.di1_stop_monitor_active = True
        monitor_thread = threading.Thread(
            target=self.di1_stop_monitor_loop,
            daemon=True,
        )
        monitor_thread.start()

    def stop_di1_stop_monitor(self):
        """Stop background DI1 monitor."""
        self.di1_stop_monitor_active = False

    def di1_stop_monitor_loop(self):
        """Stop robot motion if DI1 is triggered during motion."""
        while self.di1_stop_monitor_active and self.is_alive:
            code, read_di1_value = self.arm.get_cgpio_digital(
                CONTROLLER_DI1_NUMBER
            )
            if code != 0:
                self.print_message(
                    "DI1 stop monitor read failed, code={}".format(code)
                )
                self.alive = False
                return

            if read_di1_value == DI1_STOP_VALUE:
                self.print_message("DI1 stop triggered. Stopping robot.")
                self.di1_stop_triggered = True
                self.di1_stop_monitor_active = False
                self.arm.set_state(4)
                return

            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)

    def set_gripper_open(self):
        """Open the xArm gripper."""
        code = self.arm.set_gripper_position(
            GRIPPER_OPEN_POSITION,
            wait=True,
            speed=GRIPPER_SPEED,
            auto_enable=True,
        )
        # time.sleep(0.3)
        return self.check_code(code, "set_gripper_open")

    def set_gripper_closed(self):
        """Close the xArm gripper."""
        code = self.arm.set_gripper_position(
            GRIPPER_CLOSED_POSITION,
            wait=True,
            speed=GRIPPER_SPEED,
            auto_enable=True,
        )
        # time.sleep(0.3)
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

        if not self.move_to_pose(PLACE_ABOVE_POSE, "move_up_from_place"):
            return False

        return self.move_to_pose(PICK_ABOVE_POSE, "return_pick_above")

    def run(self):
        """Run pick-and-place each time DI0 is triggered."""
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

            if RUN_WITHOUT_DI0_WAIT:
                self.start_di1_stop_monitor()
                self.pick_and_place_cycle()
                self.stop_di1_stop_monitor()
                return

            while self.is_alive:
                if not self.wait_for_di0_start():
                    return

                if not self.wait_for_di0_release():
                    return

                self.start_di1_stop_monitor()

                if not self.pick_and_place_cycle():
                    self.stop_di1_stop_monitor()
                    if self.di1_stop_triggered:
                        if not self.go_home_after_di1_stop():
                            return
                        continue
                    return

                self.stop_di1_stop_monitor()

                if self.di1_stop_triggered:
                    if not self.go_home_after_di1_stop():
                        return
                    continue

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
