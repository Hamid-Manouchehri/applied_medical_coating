#!/usr/bin/env python3
"""Simple xArm 6 pick-and-place between two Cartesian way points.

No external buttons: run the script and, after a short countdown, the
robot runs run_sequence() by itself, once or repeatedly (see
LOOP_SEQUENCE_FOREVER). Press q at any time to stop the robot and quit.

Two building blocks, called in any order inside run_sequence():

    move_to_pose(pose, label)
        Moves the tool in a straight line (moveL) to a Cartesian pose
        [x, y, z, roll, pitch, yaw] in mm and degrees.

    set_gripper(position, label)
        Moves the gripper to any position you choose.

The sequence has four way points:

    open gripper (once, at the start) and then, every cycle:
    PICK_POSE -> close gripper -> WAYPOINT_1 -> WAYPOINT_2 ->
    PLACE_POSE -> open gripper

Inputs:
    Robot connection through ROBOT_IP.
    PICK_POSE, WAYPOINT_1, WAYPOINT_2, PLACE_POSE, each
    [x, y, z, roll, pitch, yaw] in mm/deg.
    Gripper positions (device units, 0 = closed).
    Keyboard key "q", pressed at any time, to stop and quit.

Outputs:
    Robot motion and gripper commands sent to the xArm 6.

Important:
    - If any way point is set to None the script refuses to run. Jog
      the real xArm 6 to each pose in UFactory Studio and enter the
      Cartesian values it shows. WAYPOINT_1 and WAYPOINT_2 are
      placeholders equal to the pick/place poses, so they add no
      motion until you change them.
    - There is no home point: the first move is a straight line from
      wherever the robot is now to PICK_POSE, and the way points are
      joined by straight lines. Make sure those paths are clear.
    - There is no button to confirm the start, so the robot begins
      moving START_DELAY_SECONDS after the script starts. Press q
      during that countdown to cancel.
    - q calls set_state(6), a deceleration stop: the robot ramps down
      instead of halting abruptly. It cannot be resumed; start the
      script again.
    - If any command fails (robot error, external stop, ...), the run
      ends and the robot stays where it is.
    - To keep each cycle fast, gripper commands do not wait for the
      gripper to finish: the arm moves on right after the command is
      sent (the SDK still waits for the arm to arrive first). Closing
      may therefore not be complete when the arm lifts, and opening
      may not be complete when it leaves. If the object slips or is
      dragged, set GRIPPER_SETTLE_SECONDS above 0.
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

# TODO: Way points, [x, y, z, roll, pitch, yaw] in mm and degrees, in
# the robot base frame. Read them off the real arm in UFactory Studio.
# Example: PICK_POSE = [300.0, 0.0, 200.0, 180.0, 0.0, 0.0]
PICK_POSE = [295, 185, 340, 180.0 , 0.0, 0.0]

# TODO: Two transit way points between the pick and the place, same
# format. They are placeholders equal to PICK_POSE and PLACE_POSE, so
# they add no motion yet - replace them with the poses you want.
WAYPOINT_1 = [295, -200, 340, 180.0, 0.0, 0.0]

WAYPOINT_2 = [295, -200, 600, 180.0, 0.0, 0.0]

PLACE_POSE = [295, 185, 600, 180.0 , 0.0, 0.0]

# TODO: Gripper positions, device units. Typically 0 (closed) up to
# about 850 (fully open) for the xArm gripper. Set the closed position
# a bit tighter than your object so the gripper holds it firmly.
GRIPPER_OPEN_POSITION = 100
GRIPPER_CLOSED_POSITION = 0

# TODO: Tool speed of the moves, 0.1 to 1000 mm/s. Start low.
LINEAR_SPEED = 1000

# TODO: Tool acceleration of the moves, 1 to 50000 mm/s^2.
LINEAR_ACCELERATION = 35000

# TODO: Change if the gripper closes too slowly or too hard.
GRIPPER_SPEED = 5000  # range: 0-5k

# TODO: Pause after each gripper command, in seconds. 0.0 means the
# arm moves on right after the command is sent, possibly before the
# gripper has finished closing or opening. If the object slips or gets
# dragged away, raise this (e.g. 0.3).
GRIPPER_SETTLE_SECONDS = 0.0

# TODO: Update if you know your real tool payload.
GRIPPER_MASS_KG = 0.82
GRIPPER_CENTER_OF_MASS_MM = [0, 0, 48]

# TODO: Set True to repeat run_sequence() until q is pressed or a
# command fails. Set False to run it once and finish.
LOOP_SEQUENCE_FOREVER = True

# TODO: Seconds between starting the script and the first move. Press
# q during this time to cancel.
START_DELAY_SECONDS = 0.0

# TODO: Keyboard poll interval, in seconds. Smaller reacts to q
# faster, larger uses less CPU.
KEYBOARD_READ_INTERVAL_SECONDS = 0.05


# Controller error codes 19 and 28 both mean "End Module Communication
# Error" - a gripper/end-effector RS485 comm glitch. This is usually a
# transient wiring/noise blip, not a safety fault, so it is cleared
# automatically instead of stopping the script. See error_warn_changed_
# callback() below. Any other error code still stops the script.
RECOVERABLE_ERROR_CODES = (19, 28)


def read_keyboard_key():
    """Read one keyboard key without blocking, or "" if none waiting."""
    read_ready, _, _ = select.select([sys.stdin], [], [], 0)

    if not read_ready:
        return ""

    return sys.stdin.read(1)


class PickAndPlaceRobot:
    """Pose and gripper actions you call in any order. q stops."""

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
        """Stop the script if the robot reports an error.

        Codes 19/28 (end module comm glitch, see RECOVERABLE_ERROR_
        CODES above) are cleared automatically and the script keeps
        running. Any other error code stops the script so you can
        check the robot.
        """
        if data and data["error_code"] != 0:
            if data["error_code"] in RECOVERABLE_ERROR_CODES:
                self.print_message(
                    "error={} (end module comm glitch), "
                    "auto-clearing.".format(data["error_code"])
                )
                self.arm.clean_error()
                self.arm.set_state(0)
                return

            self.alive = False
            self.print_message("error={}, quit".format(data["error_code"]))
            self.arm.release_error_warn_changed_callback(
                self.error_warn_changed_callback
            )

    def state_changed_callback(self, data):
        """Stop the script if the robot enters a stop state.

        4 is an abrupt stop, 6 is a deceleration stop. q uses 6, but
        either can also happen from outside (e.g. an E-stop or a stop
        from Studio), so both are handled the same way here.
        """
        if data and data["state"] in (4, 6):
            if self.quit_requested:
                self.print_message(
                    "state={} from q.".format(data["state"])
                )
                return

            self.alive = False
            self.print_message("state={}, quit".format(data["state"]))
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
        """Stop the robot and quit the moment q is pressed.

        Uses set_state(6), a deceleration stop, so the robot ramps
        down instead of halting abruptly.
        """
        while self.keyboard_monitor_running and self.arm.connected:
            key = read_keyboard_key()

            if key == "q":
                self.print_message("q pressed. Stopping.")
                self.quit_requested = True
                self.keyboard_monitor_running = False
                self.arm.set_state(6)
                return

            time.sleep(KEYBOARD_READ_INTERVAL_SECONDS)

    # --- Building blocks: call these in any order in run_sequence() ---

    def move_to_pose(self, pose, label):
        """Move the tool in a straight line (moveL) to a Cartesian pose.

        pose is [x, y, z, roll, pitch, yaw] in mm and degrees, in the
        robot base frame. The line starts from wherever the robot is.
        """
        if self.quit_requested:
            return False

        self.print_message("Moving linearly to {}: {}".format(label, pose))
        code = self.arm.set_position(
            *pose,
            speed=LINEAR_SPEED,
            mvacc=LINEAR_ACCELERATION,
            radius=-1.0,
            wait=False,
        )
        move_ok = self.check_code(code, label)
        return move_ok and not self.quit_requested

    def set_gripper(self, position, label, wait=False):
        """Move the gripper to any position you choose.

        The SDK first waits for the arm to finish any queued move, so
        the gripper acts when the arm has arrived. With wait=False the
        call then returns at once, without waiting for the gripper to
        finish (plus GRIPPER_SETTLE_SECONDS, if set). With wait=True it
        waits until the gripper has stopped moving, which also reports
        a gripper that does not respond.
        """
        if self.quit_requested:
            return False

        self.print_message("Gripper to {}: {}".format(label, position))
        code = self.arm.set_gripper_position(
            position,
            wait=wait,
            speed=GRIPPER_SPEED,
            auto_enable=False,
        )
        gripper_ok = self.check_code(code, label)
        time.sleep(GRIPPER_SETTLE_SECONDS)
        return gripper_ok and not self.quit_requested

    # --- Your sequence ---

    def run_sequence(self):
        """One pick-and-place cycle. Returns True if it completed.

        Starts with the gripper open (run() opens it once before the
        first cycle, and each cycle ends by opening it). Call
        move_to_pose() / set_gripper() here in any order you want.
        """
        if not self.move_to_pose(PICK_POSE, "pick"):
            return False
        if not self.set_gripper(GRIPPER_CLOSED_POSITION, "gripper_close"):
            return False
        if not self.move_to_pose(WAYPOINT_1, "waypoint_1"):
            return False
        if not self.move_to_pose(WAYPOINT_2, "waypoint_2"):
            return False
        if not self.move_to_pose(PLACE_POSE, "place"):
            return False

        return self.set_gripper(GRIPPER_OPEN_POSITION, "gripper_open")

    def wait_for_start(self):
        """Count down before the first move; q cancels."""
        self.print_message(
            "Starting in {} s. Press q to cancel.".format(
                START_DELAY_SECONDS
            )
        )
        waited_seconds = 0.0
        while (
            waited_seconds < START_DELAY_SECONDS
            and not self.quit_requested
        ):
            time.sleep(KEYBOARD_READ_INTERVAL_SECONDS)
            waited_seconds += KEYBOARD_READ_INTERVAL_SECONDS

        return not self.quit_requested

    def run(self):
        """Count down, then run run_sequence() once or repeatedly."""
        try:
            if None in (PICK_POSE, WAYPOINT_1, WAYPOINT_2, PLACE_POSE):
                self.print_message(
                    "Set PICK_POSE, WAYPOINT_1, WAYPOINT_2 and PLACE_POSE "
                    "in User Settings first."
                )
                return

            code = self.arm.set_tcp_load(
                GRIPPER_MASS_KG,
                GRIPPER_CENTER_OF_MASS_MM,
            )
            if not self.check_code(code, "set_tcp_load_gripper"):
                return

            self.start_keyboard_monitor()

            if not self.wait_for_start():
                self.print_message("Cancelled before the first move.")
                return

            # Open once, and wait for it: this also confirms that the
            # gripper responds before the fast loop starts.
            if not self.set_gripper(
                GRIPPER_OPEN_POSITION, "gripper_open", wait=True
            ):
                return

            cycle_count = 0
            while not self.quit_requested:
                if not self.run_sequence():
                    self.print_message("Sequence stopped.")
                    break

                cycle_count += 1
                self.print_message(
                    "Cycle {} complete.".format(cycle_count)
                )

                if not LOOP_SEQUENCE_FOREVER:
                    break

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
    PickAndPlaceRobot.print_message(
        "xArm-Python-SDK Version:{}".format(version.__version__)
    )

    old_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
        time.sleep(0.5)

        robot = PickAndPlaceRobot(read_write_arm)
        robot.run()

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings,
        )
        read_write_arm.disconnect()
