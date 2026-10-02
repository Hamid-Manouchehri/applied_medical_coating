#!/usr/bin/env python3
"""xArm 5 double peg extraction for offline use in UFactory Studio.

Import this file into UFactory Studio and run it on the controller -
no PC, keyboard or terminal is needed while it runs. Three pushbuttons
wired to controller digital inputs drive everything:

    DI0 button (start/stop toggle):
        Stopped -> pressed -> errors are cleared, the robot is
                               enabled, moves to
                               VIA_JOINT_START_STOP_HOME, then
                               run_sequence() starts (once, or
                               repeating - see LOOP_SEQUENCE_FOREVER
                               below).
        Running, paused or moving to the DI2 configuration ->
                               pressed -> robot decelerates to a stop
                               where it is and waits.

    DI1 button (pause/resume toggle):
        Running -> pressed -> pauses immediately, even mid-move.
        Paused  -> pressed -> resumes from where it paused.
        Ignored while stopped or moving to the DI2 configuration.

    DI2 button (move to a specific configuration):
        Pressed -> if the robot is running, it first decelerates to
                   a stop. Then errors are cleared and it moves to
                   VIA_JOINT_DI2 (joint space, with its own speed
                   DI2_ANGLE_SPEED and acceleration
                   DI2_ANGLE_ACCELERATION) and waits there, stopped.
                   Press DI0 to start again (it goes to
                   VIA_JOINT_START_STOP_HOME first). Pressing DI0
                   during the move stops the robot where it is.
                   The gripper is not touched, so a held peg stays
                   held.

Building blocks you call in any order inside run_sequence():

    move_to_via(via_angles, label)
        Moves the robot to a joint-space via point (q1..q5, degrees).

    move_linear_to_pose(pose, label)
        Moves the tool in a straight line (moveL) to a Cartesian pose
        [x, y, z, roll, pitch, yaw] in mm and degrees.

    set_gripper(position, label)
        Opens/closes the gripper to any position you choose.

Inputs:
    Controller digital inputs DI0, DI1, DI2 (pushbuttons).
    Joint-space via points in degrees, Cartesian poses in mm/degrees.
    Gripper positions (device units, 0 = closed).

Outputs:
    Robot motion and gripper commands sent to the xArm 5.

Important:
    - The script never ends on its own. It keeps waiting for the
      buttons until it is stopped from Studio or the controller is
      powered off.
    - If any command fails (robot error, external stop, ...), the run
      ends and the robot stays stopped where it is. Press DI0 to
      start again or DI2 to move to its configuration - both clear
      errors first.
    - Stop (DI0) and the DI2 move use set_state(6), a deceleration
      stop. Pause (DI1) uses set_state(3); the firmware has no
      gentler pause, so lowering ANGLE_SPEED / ANGLE_ACCELERATION is
      the only way to soften it.
    - The DI2 move goes straight to VIA_JOINT_DI2, and a start (DI0)
      first goes straight to VIA_JOINT_START_STOP_HOME, both from
      wherever the robot is. Make sure that path is clear.
    - On the xArm 5, Cartesian poses must have roll 180 and pitch 0.
    - In Studio's controller IO settings, DI0, DI1 and DI2 must stay
      "General Input". If one of them gets a built-in function (e.g.
      Offline Task, Manual Mode), the controller reacts to that button
      as well. To launch this script with a button, give a different
      DI the "Offline Task" function.
"""

import threading
import time

from xarm.wrapper import XArmAPI


# User Settings
# TODO: IP address of the xArm controller. Studio's own generated
# Python connects with the robot IP like this, also on the controller.
ROBOT_IP = "192.168.1.229"

# TODO: Via points. Joint-space ones (VIA_JOINT_*) are (q1, q2, q3, q4,
# q5) in degrees. Cartesian ones (VIA_CART_*) are [x, y, z, roll, pitch,
# yaw] in mm and degrees (xArm 5: roll 180, pitch 0). Jog the xArm 5 to
# each pose in UFactory Studio and read the values back.
# Add, remove, or rename these freely, then use them in run_sequence().
# VIA_JOINT_START_STOP_HOME is where the robot goes first on every
# start (DI0). VIA_JOINT_DI2 is where the DI2 button sends it.
VIA_JOINT_START_STOP_HOME = [1.3, -20.7, -15.2, 35.8, 91.3]
VIA_JOINT_HOME = [1.3, -2.1, -13.8, 15.8, 91.3]
VIA_JOINT_DI2 = [0.8, -15.5, -77.8, 93.2, 91.0]
VIA_CART_1 = [287.4, 7.4, 160.0, 180.0, 0.0, -90.0]
VIA_CART_2 = [287.3, 7.6, 200, 180.0, 0., -90.]
VIA_JOINT_3 = [1.0, 17.4, -34.5, 18.3, 90.9]
VIA_JOINT_4 = [1.3, -8.5, -14.0, 21.7, 91.3]

# TODO: Gripper positions, device units. Typically 0 (closed) up to
# about 850 (fully open) for the xArm gripper. Use any value you want.
GRIPPER_OPEN_POSITION = 80
GRIPPER_CLOSED_POSITION = 0

# TODO: Joint speed range is about 0.006 to 229 deg/s.
ANGLE_SPEED = 230

# TODO: Joint acceleration range is about 0.57 to 1146 deg/s^2.
ANGLE_ACCELERATION = 1000

# TODO: Joint speed of the DI2 move, 0.006 to 229 deg/s. Kept apart
# from ANGLE_SPEED because it starts from wherever the robot stopped.
DI2_ANGLE_SPEED = 30

# TODO: Joint acceleration of the DI2 move, 0.57 to 1146 deg/s^2.
DI2_ANGLE_ACCELERATION = 100

# TODO: Tool speed of linear moves (move_linear_to_pose), 0.1 to 1000
# mm/s.
LINEAR_SPEED = 1000

# TODO: Tool acceleration of linear moves, 1 to 50000 mm/s^2.
LINEAR_ACCELERATION = 50000

# TODO: Change if the gripper closes too slowly or too hard.
GRIPPER_SPEED = 5000  # range: 0-5k

# TODO: Update if you know your real tool payload.
GRIPPER_MASS_KG = 0.82
GRIPPER_CENTER_OF_MASS_MM = [0, 0, 48]

# TODO: Set True to repeat run_sequence() until stopped/paused/homed.
# Set False to run run_sequence() once per start button press.
LOOP_SEQUENCE_FOREVER = True

# TODO: DI0 is 8. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI0_NUMBER = 8

# TODO: DI1 is 9. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI1_NUMBER = 9

# TODO: DI2 is 10. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI2_NUMBER = 10

# TODO: Set to 1 if the DI0 button reads high when pressed.
DI0_BUTTON_VALUE = 0

# TODO: Set to 1 if the DI1 button reads high when pressed.
DI1_BUTTON_VALUE = 0

# TODO: Set to 1 if the DI2 button reads high when pressed.
DI2_BUTTON_VALUE = 0

# TODO: Time between digital input reads, in seconds. Also used as
# the idle/pause wait step, and doubles as simple button debounce.
DIGITAL_READ_INTERVAL_SECONDS = 0.1

# TODO: Longest wait for a deceleration stop to finish before the
# robot is re-enabled (start or DI2 right after a stop), in seconds.
DECELERATION_STOP_TIMEOUT_SECONDS = 2.0


# Run states used by the three buttons.
STATE_STOPPED = "stopped"
STATE_RUNNING = "running"
STATE_PAUSED = "paused"
STATE_DI2_MOVE = "di2_move"

# Controller error codes 19 and 28 both mean "End Module Communication
# Error" - a gripper RS485 comm glitch, usually a transient wiring/noise
# blip. These are cleared automatically; any other error stays until
# DI0 (start) or DI2 (move) is pressed.
RECOVERABLE_ERROR_CODES = (19, 28)


def print_message(message):
    """Print a message with a timestamp."""
    current_time = time.strftime("%Y-%m-%d %H:%M:%S")
    print("[{}] {}".format(current_time, message))


class ModularRobot:
    """Via-point and gripper actions, driven by the DI0/DI1/DI2 buttons.

    DI0 starts/stops, DI1 pauses/resumes, DI2 moves to a set
    configuration."""

    def __init__(self, arm):
        self.alive = True
        self.arm = arm
        self.state = STATE_STOPPED
        self.button_monitor_running = False
        self.arm.register_error_warn_changed_callback(
            self.error_warn_changed_callback
        )

    def error_warn_changed_callback(self, data):
        """Log robot errors and auto-clear gripper comm glitches.

        Only clean_error() here, no set_state(0): if the robot is
        paused at that moment, set_state(0) would resume it.
        """
        if data and data["error_code"] != 0:
            print_message("Robot error={}".format(data["error_code"]))
            if data["error_code"] in RECOVERABLE_ERROR_CODES:
                print_message("Gripper comm glitch, clearing it.")
                self.arm.clean_error()

    def check_code(self, code, label):
        """Return True if an xArm command worked; log it if not."""
        if not self.is_alive or code != 0:
            self.alive = False
            print_message(
                "{} failed, code={}, state={}, error={}".format(
                    label,
                    code,
                    self.arm.state,
                    self.arm.error_code,
                )
            )
        return self.is_alive

    @property
    def is_alive(self):
        """Return True when the robot can take commands."""
        if self.alive and self.arm.connected and self.arm.error_code == 0:
            if self.arm.state == 5:
                wait_count = 0
                while self.arm.state == 5 and wait_count < 5:
                    wait_count += 1
                    time.sleep(0.1)
            return self.arm.state < 4
        return False

    # --- Buttons: start/stop (DI0), pause/resume (DI1), DI2 move ---

    def read_digital_input(self, input_number, input_name):
        """Read one controller digital input."""
        code, read_value = self.arm.get_cgpio_digital(input_number)

        if code != 0:
            print_message(
                "{} read failed, code={}".format(input_name, code)
            )
            return None

        return read_value

    def handle_start_stop_button(self):
        """Toggle between stopped and running.

        Stopping uses set_state(6), a deceleration stop, so the robot
        ramps down instead of halting abruptly.
        """
        if self.state == STATE_STOPPED:
            print_message("Start/stop button pressed. Starting.")
            self.state = STATE_RUNNING
        else:
            print_message("Start/stop button pressed. Stopping.")
            self.state = STATE_STOPPED
            self.arm.set_state(6)

    def handle_pause_resume_button(self):
        """Toggle between running and paused.

        Uses the xArm's own pause state (set_state(3)/set_state(0)),
        so it acts immediately, even in the middle of a move.
        """
        if self.state == STATE_RUNNING:
            print_message("Pause/resume button pressed. Pausing.")
            self.state = STATE_PAUSED
            self.arm.set_state(3)
        elif self.state == STATE_PAUSED:
            print_message("Pause/resume button pressed. Resuming.")
            self.state = STATE_RUNNING
            self.arm.set_state(0)
        else:
            print_message(
                "Pause/resume button pressed while {}. "
                "Ignored.".format(self.state)
            )

    def handle_di2_button(self):
        """Ask for the DI2 move, stopping a running robot gently first.

        The state is set before set_state(6) on purpose: the running
        sequence then sees STATE_DI2_MOVE when its move is interrupted
        and hands over to go_to_di2_configuration() instead of
        reporting a failure.
        """
        if self.state == STATE_DI2_MOVE:
            print_message("DI2 pressed. Already moving there.")
            return

        print_message("DI2 pressed. Going to the DI2 configuration.")
        robot_was_moving = self.state in (STATE_RUNNING, STATE_PAUSED)
        self.state = STATE_DI2_MOVE
        if robot_was_moving:
            self.arm.set_state(6)

    def start_button_monitor(self):
        """Start the background thread that watches the buttons."""
        self.button_monitor_running = True
        monitor_thread = threading.Thread(
            target=self.button_monitor_loop,
            daemon=True,
        )
        monitor_thread.start()

    def button_monitor_loop(self):
        """Watch DI0, DI1, DI2 and react once to each new press."""
        previous_di0_value = self.read_digital_input(
            CONTROLLER_DI0_NUMBER, "DI0"
        )
        previous_di1_value = self.read_digital_input(
            CONTROLLER_DI1_NUMBER, "DI1"
        )
        previous_di2_value = self.read_digital_input(
            CONTROLLER_DI2_NUMBER, "DI2"
        )

        while self.button_monitor_running and self.arm.connected:
            di0_value = self.read_digital_input(
                CONTROLLER_DI0_NUMBER, "DI0"
            )
            di1_value = self.read_digital_input(
                CONTROLLER_DI1_NUMBER, "DI1"
            )
            di2_value = self.read_digital_input(
                CONTROLLER_DI2_NUMBER, "DI2"
            )

            di0_just_pressed = (
                di0_value == DI0_BUTTON_VALUE
                and previous_di0_value != DI0_BUTTON_VALUE
            )
            di1_just_pressed = (
                di1_value == DI1_BUTTON_VALUE
                and previous_di1_value != DI1_BUTTON_VALUE
            )
            di2_just_pressed = (
                di2_value == DI2_BUTTON_VALUE
                and previous_di2_value != DI2_BUTTON_VALUE
            )

            if di0_just_pressed:
                self.handle_start_stop_button()

            if di1_just_pressed:
                self.handle_pause_resume_button()

            if di2_just_pressed:
                self.handle_di2_button()

            previous_di0_value = di0_value
            previous_di1_value = di1_value
            previous_di2_value = di2_value
            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)

    # --- Robot preparation and the DI2 move ---

    def wait_for_deceleration_stop(self):
        """Wait until a deceleration stop, set_state(6), has finished.

        A wait=True move returns as soon as the stop begins, while the
        arm may still be slowing down. Re-enabling it right away could
        cut that ramp short and give an abrupt stop after all.
        """
        waited_seconds = 0.0
        while (
            self.arm.state == 6
            and waited_seconds < DECELERATION_STOP_TIMEOUT_SECONDS
        ):
            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)
            waited_seconds += DIGITAL_READ_INTERVAL_SECONDS

    def prepare_robot(self):
        """Clear errors and enable the robot.

        Runs before every start (DI0) and every DI2 move, so
        those buttons also recover the robot after an error or stop.
        """
        self.wait_for_deceleration_stop()
        self.arm.clean_warn()
        self.arm.clean_error()
        self.arm.motion_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        time.sleep(1)
        self.alive = True

        code = self.arm.set_tcp_load(
            GRIPPER_MASS_KG,
            GRIPPER_CENTER_OF_MASS_MM,
        )
        return self.check_code(code, "set_tcp_load_gripper")

    def go_to_di2_configuration(self):
        """Move to VIA_JOINT_DI2 (DI2), then wait stopped for DI0.

        Uses its own speed and acceleration (DI2_ANGLE_SPEED,
        DI2_ANGLE_ACCELERATION). The state stays STATE_DI2_MOVE during
        the move, so pressing DI0 stops the robot where it is (see
        handle_start_stop_button). The gripper is not touched.
        """
        if self.prepare_robot() and self.state == STATE_DI2_MOVE:
            print_message(
                "Moving to the DI2 configuration: {}".format(VIA_JOINT_DI2)
            )
            code = self.arm.set_servo_angle(
                angle=VIA_JOINT_DI2,
                speed=DI2_ANGLE_SPEED,
                mvacc=DI2_ANGLE_ACCELERATION,
                wait=True,
                radius=-1.0,
            )
            if self.check_code(code, "di2_configuration"):
                print_message("At the DI2 configuration. Press DI0.")

        if self.state == STATE_DI2_MOVE:
            self.state = STATE_STOPPED

    # --- Building blocks: call these in any order in run_sequence() ---
    # Note: no pause check is needed here. self.arm.set_state(3)/(0)
    # (see handle_pause_resume_button) pauses and resumes these SDK
    # calls directly, whether they are idle or already mid-move.
    # Moves are queued (wait=False) and return at once; a gripper call
    # makes the SDK wait for the arm to finish before it acts.

    def move_to_via(self, via_angles, label):
        """Move the robot to one joint-space via point."""
        if self.state not in (STATE_RUNNING, STATE_PAUSED):
            return False

        print_message("Moving to {}: {}".format(label, via_angles))
        code = self.arm.set_servo_angle(
            angle=via_angles,
            speed=ANGLE_SPEED,
            mvacc=ANGLE_ACCELERATION,
            wait=False,
            radius=-1.0,
        )
        move_ok = self.check_code(code, label)
        return move_ok and self.state in (STATE_RUNNING, STATE_PAUSED)

    def move_linear_to_pose(self, pose, label):
        """Move the tool in a straight line (moveL) to a Cartesian pose.

        pose is [x, y, z, roll, pitch, yaw] in mm and degrees, in the
        robot base frame. On the xArm 5, roll must be 180 and pitch 0.
        The line starts from wherever the robot is now.
        """
        if self.state not in (STATE_RUNNING, STATE_PAUSED):
            return False

        print_message("Moving linearly to {}: {}".format(label, pose))
        code = self.arm.set_position(
            *pose,
            speed=LINEAR_SPEED,
            mvacc=LINEAR_ACCELERATION,
            radius=-1.0,
            wait=False,
        )
        move_ok = self.check_code(code, label)
        return move_ok and self.state in (STATE_RUNNING, STATE_PAUSED)

    def set_gripper(self, position, label):
        """Move the gripper to any position you choose."""
        if self.state not in (STATE_RUNNING, STATE_PAUSED):
            return False

        code = self.arm.set_gripper_position(
            position,
            wait=False,
            speed=GRIPPER_SPEED,
            auto_enable=True,
        )
        gripper_ok = self.check_code(code, label)
        return gripper_ok and self.state in (STATE_RUNNING, STATE_PAUSED)

    # --- Your sequence ---

    def run_sequence(self):
        """Call move_to_via() / move_linear_to_pose() / set_gripper()
        here in any order you want. This example just demonstrates
        the pattern."""
        if not self.set_gripper(GRIPPER_OPEN_POSITION, "gripper_open"):
            return
        if not self.move_linear_to_pose(VIA_CART_1, "via_cart_1"):
            return
        if not self.set_gripper(GRIPPER_CLOSED_POSITION, "gripper_close"):
            return
        if not self.move_linear_to_pose(VIA_CART_2, "via_cart_2"):
            return
        if not self.move_to_via(VIA_JOINT_3, "via_joint_3"):
            return
        if not self.set_gripper(GRIPPER_OPEN_POSITION, "gripper_open"):
            return
        if not self.move_to_via(VIA_JOINT_4, "via_joint_4"):
            return

    def run_until_stopped(self):
        """Run run_sequence() once, or repeatedly, until stopped."""
        if not self.prepare_robot():
            print_message("Could not enable the robot. Press DI0 again.")
            if self.state == STATE_RUNNING:
                self.state = STATE_STOPPED
            return

        # Every start goes to VIA_JOINT_START_STOP_HOME first.
        if not self.move_to_via(
            VIA_JOINT_START_STOP_HOME, "via_start_stop_home"
        ):
            # Not stopped or redirected by a button, so the move failed.
            if self.state in (STATE_RUNNING, STATE_PAUSED):
                print_message(
                    "Could not reach start/stop home. Press DI0 again."
                )
                self.state = STATE_STOPPED
            return

        print_message("Running sequence.")
        while self.state in (STATE_RUNNING, STATE_PAUSED):
            if self.state == STATE_PAUSED:
                time.sleep(DIGITAL_READ_INTERVAL_SECONDS)
                continue

            self.run_sequence()

            # Stopped (DI0) or sent to the DI2 configuration by a button.
            if self.state not in (STATE_RUNNING, STATE_PAUSED):
                return

            # A command failed (robot error, external stop, ...).
            if not self.alive:
                print_message(
                    "Sequence failed. Press DI0 to start again "
                    "or DI2 to move to its configuration."
                )
                self.state = STATE_STOPPED
                return

            if not LOOP_SEQUENCE_FOREVER:
                print_message("Sequence complete. Press DI0 to rerun.")
                self.state = STATE_STOPPED
                return

            print_message("Sequence complete. Repeating.")

    def run(self):
        """Wait for the buttons and do what they ask, forever."""
        self.start_button_monitor()
        print_message("Ready. DI0 start/stop, DI1 pause, DI2 move.")

        try:
            while self.arm.connected:
                if self.state == STATE_RUNNING:
                    self.run_until_stopped()
                elif self.state == STATE_DI2_MOVE:
                    self.go_to_di2_configuration()
                else:
                    time.sleep(DIGITAL_READ_INTERVAL_SECONDS)
        finally:
            self.button_monitor_running = False
            self.arm.release_error_warn_changed_callback(
                self.error_warn_changed_callback
            )


if __name__ == "__main__":
    read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
    time.sleep(0.5)

    robot = ModularRobot(read_write_arm)

    try:
        robot.run()
    finally:
        read_write_arm.disconnect()
