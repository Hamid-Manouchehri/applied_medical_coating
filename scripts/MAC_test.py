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

Two controller digital inputs, each wired to a physical pushbutton,
control when run_sequence() executes:

    DI0 button (start/stop toggle):
        Stopped -> pressed -> robot re-enables and run_sequence()
                               starts (once, or repeating - see
                               LOOP_SEQUENCE_FOREVER below).
        Running or paused -> pressed -> robot stops immediately.

    DI1 button (pause/resume toggle):
        Running -> pressed -> pauses immediately, even mid-move.
        Paused  -> pressed -> resumes immediately from where it
                               paused.
        Ignored if the robot is stopped.

Inputs:
    Robot connection through ROBOT_IP.
    Joint-space via points in degrees.
    Gripper positions (device units, 0 = closed).
    Controller digital input DI0, start/stop pushbutton.
    Controller digital input DI1, pause/resume pushbutton.
    Keyboard key "q", pressed at any time, to quit the whole script.

Outputs:
    Robot motion and gripper commands sent to the xArm 6.

Important:
    Both the stop button and q call set_state(4), which can interrupt
    a move that is already in progress and cannot be resumed - after
    a stop, run_sequence() starts over from the top.

    Pause uses the xArm's own pause state, set_state(3). This
    interrupts a move immediately, mid-trajectory, and resume
    (set_state(0)) continues that same move from where it paused -
    unlike stop, nothing is lost or restarted.

    q quits the whole script. The stop button only stops the current
    run; pressing start/stop again re-enables the robot and starts
    run_sequence() again without needing to restart the script.
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
VIA_HOME = [2.6, -43.0, -42.0, 0.0, 84.0, 6.0]
VIA_POINT_1 = [2.6, -40.0, -53.0, 0.0, 90.0, 5.0]
VIA_POINT_2 = [50.8, -12.0, -57.0, 75.0, 89.0, 5.0]

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

# TODO: Set True to repeat run_sequence() until stopped/paused/quit.
# Set False to run run_sequence() once per start button press.
LOOP_SEQUENCE_FOREVER = True

# TODO: DI0 is 8. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI0_NUMBER = 8

# TODO: DI1 is 9. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI1_NUMBER = 9

# TODO: Set to 1 if the DI0 button reads high when pressed.
DI0_BUTTON_VALUE = 0

# TODO: Set to 1 if the DI1 button reads high when pressed.
DI1_BUTTON_VALUE = 0

# TODO: Time between digital input reads, in seconds. Also used as
# the idle/pause wait step, and doubles as simple button debounce.
DIGITAL_READ_INTERVAL_SECONDS = 0.1


# Robot run states used by the start/stop and pause/resume buttons.
STATE_STOPPED = "stopped"
STATE_RUNNING = "running"
STATE_PAUSED = "paused"


def read_keyboard_key():
    """Read one keyboard key without blocking, or "" if none waiting."""
    read_ready, _, _ = select.select([sys.stdin], [], [], 0)

    if not read_ready:
        return ""

    return sys.stdin.read(1)


class ModularRobot:
    """Via-point and gripper actions you call in any order.

    DI0 starts/stops, DI1 pauses/resumes, q quits."""

    def __init__(self, arm):
        self.alive = True
        self.arm = arm
        self.state = STATE_STOPPED
        self.quit_requested = False
        self.stop_button_active = False
        self.keyboard_monitor_running = False
        self.button_monitor_running = False
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
            if self.stop_button_active or self.quit_requested:
                self.print_message("state=4 from stop button or q.")
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

    # --- Keyboard "q to quit" ---

    def start_keyboard_monitor(self):
        """Start the background thread that watches for q."""
        self.keyboard_monitor_running = True
        monitor_thread = threading.Thread(
            target=self.keyboard_monitor_loop,
            daemon=True,
        )
        monitor_thread.start()

    def keyboard_monitor_loop(self):
        """Quit the whole script the moment q is pressed."""
        while self.keyboard_monitor_running and self.arm.connected:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Quitting.")
                self.quit_requested = True
                self.state = STATE_STOPPED
                self.keyboard_monitor_running = False
                self.arm.set_state(4)
                return

            time.sleep(KEYBOARD_READ_INTERVAL_SECONDS)

    # --- Start/stop and pause/resume buttons ---

    def read_digital_input(self, input_number, input_name):
        """Read one controller digital input."""
        code, read_value = self.arm.get_cgpio_digital(input_number)

        if code != 0:
            self.print_message(
                "{} read failed, code={}".format(input_name, code)
            )
            return None

        return read_value

    def handle_start_stop_button(self):
        """Toggle between stopped and running."""
        if self.state == STATE_STOPPED:
            self.print_message("Start/stop button pressed. Starting.")
            self.state = STATE_RUNNING
        else:
            self.print_message("Start/stop button pressed. Stopping.")
            self.stop_button_active = True
            self.state = STATE_STOPPED
            self.arm.set_state(4)

    def handle_pause_resume_button(self):
        """Toggle between running and paused.

        Uses the xArm's own pause state (set_state(3)/set_state(0))
        so the effect is immediate, even in the middle of a move,
        instead of waiting for the current via point to finish.
        """
        if self.state == STATE_RUNNING:
            self.print_message("Pause/resume button pressed. Pausing.")
            self.state = STATE_PAUSED
            self.arm.set_state(3)
        elif self.state == STATE_PAUSED:
            self.print_message("Pause/resume button pressed. Resuming.")
            self.state = STATE_RUNNING
            self.arm.set_state(0)
        else:
            self.print_message(
                "Pause/resume button pressed while stopped. Ignored."
            )

    def start_button_monitor(self):
        """Start the background thread that watches both buttons."""
        self.button_monitor_running = True
        monitor_thread = threading.Thread(
            target=self.button_monitor_loop,
            daemon=True,
        )
        monitor_thread.start()

    def button_monitor_loop(self):
        """Watch DI0 and DI1 and trigger a toggle on each new press."""
        previous_di0_value = self.read_digital_input(
            CONTROLLER_DI0_NUMBER, "DI0"
        )
        previous_di1_value = self.read_digital_input(
            CONTROLLER_DI1_NUMBER, "DI1"
        )

        while self.button_monitor_running and self.arm.connected:
            di0_value = self.read_digital_input(
                CONTROLLER_DI0_NUMBER, "DI0"
            )
            di1_value = self.read_digital_input(
                CONTROLLER_DI1_NUMBER, "DI1"
            )

            di0_just_pressed = (
                di0_value == DI0_BUTTON_VALUE
                and previous_di0_value != DI0_BUTTON_VALUE
            )
            di1_just_pressed = (
                di1_value == DI1_BUTTON_VALUE
                and previous_di1_value != DI1_BUTTON_VALUE
            )

            if di0_just_pressed:
                self.handle_start_stop_button()

            if di1_just_pressed:
                self.handle_pause_resume_button()

            previous_di0_value = di0_value
            previous_di1_value = di1_value
            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)

    # --- Building blocks: call these in any order in run_sequence() ---
    # Note: no pause check is needed here. self.arm.set_state(3)/(0)
    # (see handle_pause_resume_button) pauses and resumes these SDK
    # calls directly, whether they are idle or already mid-move.

    def move_to_via(self, via_angles, label):
        """Move the robot to one joint-space via point."""
        if self.quit_requested or self.state == STATE_STOPPED:
            return False

        code = self.arm.set_servo_angle(
            angle=via_angles,
            speed=ANGLE_SPEED,
            mvacc=ANGLE_ACCELERATION,
            wait=True,
            radius=-1.0,
        )
        move_ok = self.check_code(code, label)
        return (
            move_ok
            and not self.quit_requested
            and self.state != STATE_STOPPED
        )

    def set_gripper(self, position, label):
        """Move the gripper to any position you choose."""
        if self.quit_requested or self.state == STATE_STOPPED:
            return False

        code = self.arm.set_gripper_position(
            position,
            wait=True,
            speed=GRIPPER_SPEED,
            auto_enable=True,
        )
        gripper_ok = self.check_code(code, label)
        return (
            gripper_ok
            and not self.quit_requested
            and self.state != STATE_STOPPED
        )

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
        if not self.move_to_via(VIA_POINT_1, "via_point_2"):
            return
        self.move_to_via(VIA_HOME, "via_home")

    def prepare_robot_for_run(self):
        """Re-enable the robot before run_sequence() starts.

        Runs both on the very first start and on every restart after
        a stop, so the start/stop button alone is enough to recover.
        """
        self.print_message("Preparing robot to start...")
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)

        self.alive = True
        self.stop_button_active = False
        return True

    def run(self):
        """Watch the buttons and q, running run_sequence() when told."""
        try:
            code = self.arm.set_tcp_load(
                GRIPPER_MASS_KG,
                GRIPPER_CENTER_OF_MASS_MM,
            )
            if not self.check_code(code, "set_tcp_load_gripper"):
                return

            self.start_keyboard_monitor()
            self.start_button_monitor()
            self.print_message(
                "Waiting for start/stop button (DI0). Press q to quit."
            )

            while self.arm.connected and not self.quit_requested:
                if self.state != STATE_RUNNING:
                    time.sleep(DIGITAL_READ_INTERVAL_SECONDS)
                    continue

                if not self.prepare_robot_for_run():
                    self.state = STATE_STOPPED
                    continue

                while (
                    self.state != STATE_STOPPED
                    and not self.quit_requested
                ):
                    if self.state == STATE_PAUSED:
                        time.sleep(DIGITAL_READ_INTERVAL_SECONDS)
                        continue

                    self.run_sequence()

                    if self.quit_requested or self.state == STATE_STOPPED:
                        break

                    if not LOOP_SEQUENCE_FOREVER:
                        self.state = STATE_STOPPED
                        break

                    self.print_message(
                        "Sequence complete. Repeating (loop enabled)."
                    )

                if not self.quit_requested:
                    self.print_message(
                        "Stopped. Waiting for start/stop button (DI0)."
                    )

        except Exception as error:
            self.print_message("MainException: {}".format(error))
        finally:
            self.keyboard_monitor_running = False
            self.button_monitor_running = False
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
