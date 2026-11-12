#!/usr/bin/env python3
"""Simple xArm 6 pick-and-place with a Dynamixel motor on the tool bus.

No external buttons: run the script and, after a short countdown, the
robot runs run_sequence() by itself, once or repeatedly (see
LOOP_SEQUENCE_FOREVER). Press q at any time to stop the robot and quit.

Building blocks, called in any order inside run_sequence():

    move_to_pose(pose, label, wait=False)
        Moves the tool in a straight line (moveL) to a Cartesian pose
        [x, y, z, roll, pitch, yaw] in mm and degrees.

    set_gripper(position, label)
        Moves the UFactory gripper to any position you choose.

    move_motor(position, label, speed=None, wait=False)
        Sends the Dynamixel RX-24F to an absolute position 0..1023
        (1 step = 0.293 deg, 1023 steps = 300 deg) at a speed 1..1023.

The Dynamixel is reached as in x-armtest2.py:
    PC --Ethernet--> xArm --tool RS-485--> STM32 (Modbus ID 10) -->
    Dynamixel RX-24F
The UFactory gripper (Modbus ID 8) is on the same tool RS-485 bus.

The sequence:

    open gripper (once, at the start) and then, every cycle:
    PICK_POSE -> close gripper -> [motor to MOTOR_POSITION_1] ->
    WAYPOINT_1 -> WAYPOINT_2 -> PLACE_POSE -> open gripper ->
    [motor to MOTOR_POSITION_2]

Moving the motor while the arm moves:
    The tool bus is separate from the arm's motion queue, so a motor
    command can run while the arm moves. move_to_pose() and
    move_motor() return at once (wait=False) and the move runs on its
    own. The arm's moves are queued, so the script runs ahead of the
    arm: a motor command starts together with the arm move queued
    right after it only when the arm is idle at that moment. A gripper
    command waits for the arm to arrive, so the arm is idle right
    after one. For other timing use wait=True, which makes a move
    block until it has finished, or put a move_motor(..., wait=True).

Inputs:
    Robot connection through ROBOT_IP.
    PICK_POSE, WAYPOINT_1, WAYPOINT_2, PLACE_POSE, each
    [x, y, z, roll, pitch, yaw] in mm/deg.
    Gripper positions (device units, 0 = closed).
    MOTOR_POSITION_1/2 (0..1023) and MOTOR_SPEED (1..1023).
    Keyboard key "q", pressed at any time, to stop and quit.

Outputs:
    Robot, gripper and motor commands sent through the xArm 6.

Important:
    - If any way point is set to None the script refuses to run. Jog
      the real xArm 6 to each pose in UFactory Studio and enter the
      Cartesian values it shows.
    - MOTOR_POSITION_1/2 start as None, which skips the motor move.
      At start-up the script prints the motor's current position; set
      the targets from that. Check the motor's travel is clear first.
    - At start-up the tool bus baud rate is set to MODBUS_BAUDRATE if
      it differs, which reboots the tool module. Do not change it
      while the arm is moving. Set USE_DYNAMIXEL = False to leave the
      tool bus alone and run the arm and gripper only.
    - If the STM32 or the Dynamixel does not answer at start-up, the
      script refuses to run. If a motor command fails later, the arm
      is stopped with a deceleration stop and the run ends.
    - There is no home point: the first move is a straight line from
      wherever the robot is now to PICK_POSE, and the way points are
      joined by straight lines. Make sure those paths are clear.
    - There is no button to confirm the start, so the robot begins
      moving START_DELAY_SECONDS after the script starts. Press q
      during that countdown to cancel.
    - q calls set_state(6), a deceleration stop: the robot ramps down
      instead of halting abruptly, and the motor is told to hold its
      current position. It cannot be resumed; start the script again.
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

# TODO: Set True to drive the Dynamixel motor from this script. Set
# False to run the arm and gripper alone (the tool bus is not touched).
USE_DYNAMIXEL = True

# TODO: Motor targets, absolute position 0..1023 (1 step = 0.293 deg,
# 1023 steps = 300 deg). None skips the move. MOTOR_POSITION_1 is sent
# right after the pick, MOTOR_POSITION_2 right after the place (see
# run_sequence()). The motor's position is printed at start-up.
MOTOR_POSITION_1 = 10
MOTOR_POSITION_2 = 20

# TODO: Motor speed, 1..1023 (1 unit is about 0.111 rpm, 100 = ~11 rpm).
MOTOR_SPEED = 10

# TODO: Longest wait for a motor move when wait=True, in seconds.
MOTOR_MOVE_TIMEOUT_SECONDS = 30

# TODO: Modbus ID of the STM32 (the UFactory gripper is 8).
STM_ID = 10

# TODO: Tool RS-485 baud rate, same as USART2 in the STM32 (and the
# UFactory gripper's default).
MODBUS_BAUDRATE = 2000000

# TODO: Tool RS-485 Modbus reply timeout, in milliseconds.
MODBUS_TIMEOUT_MS = 50

# TODO: How many times to try reaching the STM32 at start-up before
# giving up (errors are cleared before each try).
MOTOR_SETUP_ATTEMPTS = 5

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

# STM32 registers, from x-armtest2.py.
TOOL_RS485_HOST_ID = 9   # tool-end RS-485 port, not a device ID
REG_SPEED = 4
REG_MOVE_ABS = 6
REG_SCAN_STATE = 10      # 10 scan state, 11 id, 12 baud/100, 13 progress,
                         # 14 spinning, 15 ready
REG_POSITION = 16        # 16 pos, 17 valid, 18 moving, 19 goal, 20 speed,
                         # 21 mode, 22 clamped
SCAN_RUNNING = 1
EXCEPTIONS = {
    1: "illegal function",
    2: "illegal register",
    3: "illegal value",
    6: "busy (scan running)",
}


class ModbusError(Exception):
    """A tool-bus Modbus request to the STM32 failed."""


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
                self.stop_motor()
                return

            time.sleep(KEYBOARD_READ_INTERVAL_SECONDS)

    # --- Dynamixel motor on the tool RS-485 bus (STM32 behind it) ---

    def motor_modbus(self, frame):
        """Send a Modbus frame to the STM32 (the xArm adds the CRC).

        Returns the reply as a list of bytes. Raises ModbusError.
        """
        code, reply = self.arm.getset_tgpio_modbus_data(
            frame, host_id=TOOL_RS485_HOST_ID
        )
        reply = list(reply) if reply else []

        if code == 1:
            raise ModbusError(
                "the controller has an uncleared error (C{}), so it "
                "refused the request (code 1)".format(self.arm.error_code)
            )

        if code != 0:
            raise ModbusError(
                "no reply from STM32 (code {}) - check A/B, GND, "
                "baud".format(code)
            )

        # An exception reply looks like: ID, function | 0x80, code.
        for i in range(len(reply) - 2):
            if reply[i] == STM_ID and reply[i + 1] in (0x83, 0x86):
                raise ModbusError(
                    "STM32 refused: {}".format(
                        EXCEPTIONS.get(reply[i + 2], reply[i + 2])
                    )
                )

        return reply

    def motor_write(self, register, value):
        """Write one STM32 register."""
        value &= 0xFFFF
        self.motor_modbus(
            [STM_ID, 0x06, register >> 8, register & 0xFF,
             value >> 8, value & 0xFF]
        )

    def motor_read(self, start, quantity):
        """Read STM32 registers, return them as a list of numbers."""
        reply = self.motor_modbus(
            [STM_ID, 0x03, start >> 8, start & 0xFF, 0, quantity]
        )

        for i in range(len(reply) - 2):
            if (
                reply[i] == STM_ID
                and reply[i + 1] == 0x03
                and reply[i + 2] == quantity * 2
            ):
                data = reply[i + 3:i + 3 + quantity * 2]
                if len(data) == quantity * 2:
                    return [
                        (data[j] << 8) | data[j + 1]
                        for j in range(0, len(data), 2)
                    ]

        raise ModbusError("unexpected reply {}".format(reply))

    def read_motor(self):
        """Read the motor state: position, valid, moving, goal, ..."""
        values = self.motor_read(REG_POSITION, 7)
        keys = (
            "pos", "valid", "moving", "goal", "speed", "mode", "clamped"
        )
        return dict(zip(keys, values))

    def clear_controller_errors(self):
        """Clear errors and warnings and re-enable the arm.

        Needed after the tool module reboots: that raises error C19
        (end module communication), and while any error is active the
        controller refuses tool-bus requests with code 1.
        """
        for _ in range(10):
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.motion_enable(True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            time.sleep(0.3)

            _, error_warn = self.arm.get_err_warn_code()
            if error_warn[0] == 0:
                return True

        return False

    def read_motor_setup(self):
        """Read the STM32 scan state and the motor state.

        Returns (servo_id, baud100, ready, motion). Raises ModbusError.
        """
        # Right after power-up the STM32 may still be scanning.
        for _ in range(100):
            state, servo_id, baud100, _, _, ready = self.motor_read(
                REG_SCAN_STATE, 6
            )
            if state != SCAN_RUNNING:
                break
            time.sleep(0.3)

        return servo_id, baud100, ready, self.read_motor()

    def init_motor(self):
        """Set up the tool bus and check the STM32 and Dynamixel answer.

        Runs once at start-up, before any motion: changing the baud rate
        reboots the tool module, which raises a controller error, so the
        errors are cleared afterwards and the first requests are retried.
        Returns False if the motor is not ready.
        """
        if self.arm.get_tgpio_modbus_baudrate()[1] != MODBUS_BAUDRATE:
            self.print_message(
                "Setting the tool bus baud rate to {}.".format(
                    MODBUS_BAUDRATE
                )
            )
            self.arm.set_tgpio_modbus_baudrate(MODBUS_BAUDRATE)
            time.sleep(1)
        self.arm.set_tgpio_modbus_timeout(MODBUS_TIMEOUT_MS)

        for attempt in range(1, MOTOR_SETUP_ATTEMPTS + 1):
            self.clear_controller_errors()
            try:
                servo_id, baud100, ready, motion = self.read_motor_setup()
                break
            except ModbusError as error:
                self.print_message(
                    "Motor setup attempt {} of {} failed: {}".format(
                        attempt, MOTOR_SETUP_ATTEMPTS, error
                    )
                )
                time.sleep(0.5)
        else:
            self.print_message(
                "Motor setup failed. Check the tool bus wiring, or set "
                "USE_DYNAMIXEL = False."
            )
            return False

        if not ready or not motion["valid"]:
            self.print_message(
                "Dynamixel is not answering (servo ID {}, baud {}). "
                "Check its wiring, or set USE_DYNAMIXEL = False.".format(
                    servo_id, baud100 * 100
                )
            )
            return False

        self.print_message(
            "Dynamixel ready: ID {}, baud {}, position {} ({:.1f} "
            "deg).".format(
                servo_id,
                baud100 * 100,
                motion["pos"],
                motion["pos"] * 300.0 / 1023.0,
            )
        )
        return True

    def init_tool(self):
        """Set up the tool bus: motor first, then the gripper.

        The gripper is enabled here once, because set_gripper() does not
        enable it. Returns False if the motor is not ready.
        """
        if USE_DYNAMIXEL and not self.init_motor():
            return False

        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_enable(True)
        return True

    def stop_motor(self):
        """Tell the motor to hold its current position (best effort).

        Used when q is pressed or a run fails. The motor's goal is set
        to where it is now, so a move in progress is cut short.
        """
        if not USE_DYNAMIXEL:
            return

        try:
            motion = self.read_motor()
            if motion["valid"]:
                self.motor_write(REG_MOVE_ABS, motion["pos"])
        except ModbusError as error:
            self.print_message("Could not hold the motor: {}".format(error))

    # --- Building blocks: call these in any order in run_sequence() ---

    def move_motor(self, position, label, speed=None, wait=False):
        """Send the Dynamixel to an absolute position, 0..1023.

        With wait=False the call returns as soon as the command is sent
        and the motor moves on its own, while the arm keeps moving. With
        wait=True it waits until the motor has arrived. A new command
        replaces the goal of a move still in progress. A position of
        None, or USE_DYNAMIXEL = False, skips the move.
        """
        if self.quit_requested:
            return False

        if not USE_DYNAMIXEL or position is None:
            self.print_message("Motor {}: skipped.".format(label))
            return True

        speed = MOTOR_SPEED if speed is None else speed
        self.print_message(
            "Motor to {}: position {}, speed {}".format(
                label, position, speed
            )
        )

        try:
            self.motor_write(REG_SPEED, speed)
            self.motor_write(REG_MOVE_ABS, position)
            if wait:
                self.wait_for_motor(label)
        except ModbusError as error:
            self.print_message(
                "Motor {} failed: {}. Stopping the arm.".format(
                    label, error
                )
            )
            self.alive = False
            self.arm.set_state(6)
            return False

        return not self.quit_requested

    def wait_for_motor(self, label):
        """Wait until the motor has arrived, or the timeout passes."""
        time.sleep(0.15)  # let the servo start and the STM32 refresh
        started = time.time()

        while not self.quit_requested:
            if not self.read_motor()["moving"]:
                return

            if time.time() - started > MOTOR_MOVE_TIMEOUT_SECONDS:
                self.print_message(
                    "Motor {} still moving after {} s (blocked? "
                    "overloaded?).".format(
                        label, MOTOR_MOVE_TIMEOUT_SECONDS
                    )
                )
                return

            time.sleep(0.1)

    def move_to_pose(self, pose, label, wait=False):
        """Move the tool in a straight line (moveL) to a Cartesian pose.

        pose is [x, y, z, roll, pitch, yaw] in mm and degrees, in the
        robot base frame. The line starts from wherever the robot is.
        With wait=False the move is queued and the call returns at once;
        with wait=True it returns when the arm has arrived.
        """
        if self.quit_requested:
            return False

        self.print_message("Moving linearly to {}: {}".format(label, pose))
        code = self.arm.set_position(
            *pose,
            speed=LINEAR_SPEED,
            mvacc=LINEAR_ACCELERATION,
            radius=-1.0,
            wait=wait,
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
        move_to_pose() / set_gripper() / move_motor() here in any order
        you want.
        """
        if not self.move_to_pose(PICK_POSE, "pick"):
            return False
        if not self.set_gripper(GRIPPER_CLOSED_POSITION, "gripper_close"):
            return False

        # The arm is idle here (the gripper waited for it), so the motor
        # starts together with the arm's trip towards the place pose.
        if not self.move_motor(MOTOR_POSITION_1, "motor_1"):
            return False
        if not self.move_to_pose(WAYPOINT_1, "waypoint_1"):
            return False
        if not self.move_to_pose(WAYPOINT_2, "waypoint_2"):
            return False
        if not self.move_to_pose(PLACE_POSE, "place"):
            return False
        if not self.set_gripper(GRIPPER_OPEN_POSITION, "gripper_open"):
            return False

        # The arm is idle here too, so this runs while it goes back to
        # the pick pose at the start of the next cycle.
        return self.move_motor(MOTOR_POSITION_2, "motor_2")

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

            if not self.init_tool():
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
                    if not self.quit_requested:
                        self.stop_motor()
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
