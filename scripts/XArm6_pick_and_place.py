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

The Dynamixel is wired directly to the xArm tool RS-485, as in
x-armtest4.py (no STM32):
    PC --Ethernet--> xArm --tool RS-485, transparent mode--> RX-24F
The xArm sends our bytes out unchanged, so this script builds the
Dynamixel Protocol 1.0 packets itself. The UFactory gripper (Modbus ID
8) is on the same bus and is still driven by the SDK.

Wiring (xArm 12-pin tool connector):
    pin 5    RS485-A (pink)   -> Dynamixel D+
    pin 6    RS485-B (yellow) -> Dynamixel D-
    pin 3/4  GND              -> Dynamixel GND and the 12 V supply GND
    Dynamixel VDD             -> 12 V from the LM2596 (NEVER the 24 V!)

The sequence:

    open gripper (once, at the start) and then, every cycle:
    PICK_POSE -> close gripper -> [motor to MOTOR_POSITION_1] ->
    WAYPOINT_1 -> WAYPOINT_2 -> PLACE_POSE -> open gripper ->
    [motor to MOTOR_POSITION_2]

Moving the motor while the arm moves:
    The motor traffic uses the controller's own tool-bus port (503),
    separate from the arm's motion queue, so a motor command can run
    while the arm moves. move_to_pose() and move_motor() return at once
    (wait=False) and the move runs on its own. The arm's moves are
    queued, so the script runs ahead of the arm: a motor command starts
    together with the arm move queued right after it only when the arm
    is idle at that moment. A gripper command waits for the arm to
    arrive, so the arm is idle right after one. For other timing use
    wait=True, which makes a move block until it has finished, or put
    a move_motor(..., wait=True).

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
    - A MOTOR_POSITION set to None skips that motor move. At start-up
      the script prints the motor's current position; use it to choose
      the targets. Check the motor's travel is clear first.
    - At start-up the tool bus is set to BUS_BAUD and the transparent
      mode reply timeout to TIMEOUT_MS. If the baud rate differs this
      reboots the tool module, so it is done once, before any motion.
      The servo itself must already be at BUS_BAUD. It ships at 57600,
      so run x-armtest4.py once and type 'setup' (gripper unplugged);
      this script never changes the servo's own baud rate.
    - Transparent mode needs xArm firmware 1.11 or newer and a recent
      SDK (pip install -U xarm-python-sdk). Set USE_DYNAMIXEL = False
      to leave the tool bus alone and run the arm and gripper only.
    - A servo that does not answer in time makes the controller raise
      error C19, which can interrupt an arm move in progress. The
      script clears C19/C28 and repeats the request (DXL_RETRIES), but
      if it keeps happening, run 'bustest' in x-armtest4.py and see its
      notes about 1 Mbps.
    - If the Dynamixel does not answer at start-up, the script refuses
      to run. If a motor command fails later, the arm is stopped with a
      deceleration stop and the run ends.
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
WAYPOINT_2 = [295, -200, 340, 180.0, 0.0, 40.0]
WAYPOINT_3 = [295, -200,600, 180.0, 0.0,  40.0]
WAYPOINT_4 = [295, -200, 600, 180.0, 0.0, -40.0]

PLACE_POSE = [295, 185, 600, 180.0 , 0.0, 0.0]

# TODO: Gripper positions, device units. Typically 0 (closed) up to
# about 850 (fully open) for the xArm gripper. Set the closed position
# a bit tighter than your object so the gripper holds it firmly.
GRIPPER_OPEN_POSITION = 100
GRIPPER_CLOSED_POSITION = 0

# TODO: Tool speed of the moves, 0.1 to 1000 mm/s. Start low.
LINEAR_SPEED = 1000

# TODO: Tool acceleration of the moves, 1 to 50000 mm/s^2.
LINEAR_ACCELERATION = 45000

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
MOTOR_POSITION_2 = 100

# TODO: Motor speed, 1..1023 (1 unit is about 0.111 rpm, 100 = ~11 rpm).
MOTOR_SPEED = 80

# TODO: Longest wait for a motor move when wait=True, in seconds.
MOTOR_MOVE_TIMEOUT_SECONDS = 30

# TODO: Dynamixel ID.
DXL_ID = 1

# TODO: Tool RS-485 baud rate. The Dynamixel must already be at this
# baud (see 'setup' in x-armtest4.py). 2000000 is also the UFactory
# gripper's default, so the gripper is left untouched.
BUS_BAUD = 2000000

# TODO: Reply timeout of the xArm's transparent mode, in milliseconds.
TIMEOUT_MS = 100

# TODO: Quiet gap before each message to the servo, in milliseconds.
TX_GAP_MS = 10

# TODO: How many extra times a servo request is repeated when it gets
# no valid answer.
DXL_RETRIES = 2

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

# Dynamixel Protocol 1.0, from x-armtest4.py.
DXL_PING, DXL_READ, DXL_WRITE = 0x01, 0x02, 0x03

# RX-24F control table addresses.
ADDR_CW_LIMIT = 6
ADDR_CCW_LIMIT = 8
ADDR_TORQUE = 24
ADDR_GOAL = 30
ADDR_SPEED = 32
ADDR_PRESENT = 36  # 36..46: position, speed, load, voltage, temperature,
                   # ..., moving

DXL_ERROR_NAMES = [
    "voltage", "angle limit", "overheat", "range",
    "checksum", "overload", "instruction",
]
STEP_DEG = 300.0 / 1023.0


def dxl_packet(instruction, params=()):
    """Build a Dynamixel Protocol 1.0 packet: FF FF ID LEN INSTR ... CHK."""
    body = [DXL_ID, len(params) + 2, instruction, *params]
    return [0xFF, 0xFF, *body, (~sum(body)) & 0xFF]


def dxl_parse(buffer, count):
    """Find a valid status packet in buffer.

    Returns (error_flags, data) with count data bytes, or None.
    """
    for i in range(len(buffer) - 5):
        if (
            buffer[i] != 0xFF
            or buffer[i + 1] != 0xFF
            or buffer[i + 2] != DXL_ID
        ):
            continue

        length = buffer[i + 3]
        end = i + 4 + length
        if length < 2 or end > len(buffer):
            continue

        packet = buffer[i + 2:end]  # ID LEN ERR data... CHK
        if (~sum(packet[:-1])) & 0xFF != packet[-1]:
            continue

        data = packet[3:-1]
        if len(data) >= count:
            return packet[2], data[:count]

    # Fallback: at 2 Mbps the xArm sometimes drops the leading FF FF.
    want = count + 2
    for i in range(len(buffer) - want - 1):
        if buffer[i] == DXL_ID and buffer[i + 1] == want:
            packet = buffer[i:i + want + 2]
            if (
                len(packet) == want + 2
                and (~sum(packet[:-1])) & 0xFF == packet[-1]
            ):
                return packet[2], packet[3:3 + count]

    return None


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
        self.motor_mode = None  # "joint", "wheel" or None (not read yet)
        # One servo request at a time: the q thread must not interleave
        # its hold command with a move from the main thread.
        self.motor_lock = threading.RLock()
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

    # --- Dynamixel motor on the tool RS-485 bus (wired directly) ---

    def clear_controller_errors(self):
        """Clear errors and warnings and re-enable the arm.

        Used at start-up, before any motion. Changing the tool bus baud
        rate reboots the tool module, which raises error C19, and while
        any error is active the controller refuses tool-bus requests.
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

    def clear_tool_error(self):
        """Clear a tool-bus error after a servo request got no answer.

        A device that does not answer makes the controller raise C19
        (or C28), and while it is active every tool-bus request is
        refused with code 1. Any other error is a real fault and is
        left alone. The error callback clears and re-enables the arm
        for the same codes; this makes sure the retry finds it cleared.
        """
        time.sleep(0.15)  # let the error report arrive

        if self.arm.error_code in RECOVERABLE_ERROR_CODES:
            self.arm.clean_error()
            self.arm.clean_warn()

    def set_bus(self):
        """Set the tool bus baud rate and the transparent-mode timeout.

        Changing the baud rate reboots the tool module (error C19), so
        this is done once at start-up. Returns False if it fails.
        """
        for _ in range(3):
            self.clear_controller_errors()

            code = self.arm.set_tgpio_modbus_baudrate(BUS_BAUD)
            if code != 0:
                continue

            try:
                code = self.arm.set_tgpio_modbus_timeout(
                    timeout=TIMEOUT_MS,
                    is_transparent_transmission=True,
                )
            except TypeError:
                self.print_message(
                    "This xArm SDK has no transparent mode: run "
                    "pip install -U xarm-python-sdk"
                )
                return False

            if code == 0:
                time.sleep(0.2)
                return True

        return False

    def dxl_raw(self, data, expect=0):
        """Send raw bytes through the xArm's transparent mode.

        Returns (code, reply bytes). If the request failed, the tool-bus
        error it caused is cleared so the next request is accepted.
        """
        with self.motor_lock:
            time.sleep(TX_GAP_MS / 1000)  # quiet gap between messages

            code, reply = self.arm.getset_tgpio_modbus_data(
                datas=list(data),
                min_res_len=expect,
                is_transparent_transmission=True,
                use_503_port=True,
            )
            reply = list(reply) if reply else []

            if code != 0:
                self.clear_tool_error()

            return code, reply

    def dxl_request(self, instruction, params=(), count=0,
                    retries=DXL_RETRIES):
        """Send one Dynamixel instruction and return the reply data.

        Returns the count data bytes of the status packet (an empty list
        for instructions without data), or None if there was no valid
        answer after the retries. The controller's own code is ignored:
        at 2 Mbps it sometimes reports a short reply although the packet
        is usable.
        """
        packet = dxl_packet(instruction, params)

        for _ in range(retries + 1):
            _, reply = self.dxl_raw(packet, 6 + count)
            parsed = dxl_parse(reply, count)

            if parsed:
                error_flags, data = parsed
                if error_flags:
                    names = [
                        DXL_ERROR_NAMES[bit]
                        for bit in range(7)
                        if error_flags & (1 << bit)
                    ]
                    self.print_message(
                        "Dynamixel error: {}".format(", ".join(names))
                    )
                return data

        return None

    def dxl_write8(self, address, value):
        """Write one byte to the servo. Returns True if it answered."""
        params = [address, value & 0xFF]
        return self.dxl_request(DXL_WRITE, params) is not None

    def dxl_write16(self, address, value):
        """Write two bytes to the servo. Returns True if it answered."""
        params = [address, value & 0xFF, (value >> 8) & 0xFF]
        return self.dxl_request(DXL_WRITE, params) is not None

    def dxl_read(self, address, count):
        """Read count bytes from the servo, or None if it did not answer."""
        return self.dxl_request(DXL_READ, [address, count], count)

    def get_motor_position(self):
        """Return the present position 0..1023, or None."""
        data = self.dxl_read(ADDR_PRESENT, 2)
        return None if data is None else data[0] | (data[1] << 8)

    def read_motor_mode(self):
        """Return "wheel" (continuous rotation), "joint" or None."""
        data = self.dxl_read(ADDR_CW_LIMIT, 4)
        if data is None:
            return None

        cw_limit = data[0] | (data[1] << 8)
        ccw_limit = data[2] | (data[3] << 8)
        return "wheel" if cw_limit == 0 and ccw_limit == 0 else "joint"

    def use_joint_mode(self):
        """Put the servo in position (joint) mode, without a jump, and
        switch its torque on. Returns False if it stopped answering."""
        all_ok = True

        if self.motor_mode != "joint":
            all_ok = self.dxl_write16(ADDR_SPEED, 0)  # stop the wheel
            position = self.get_motor_position()
            if position is not None:
                # Hold the present position.
                all_ok = self.dxl_write16(ADDR_GOAL, position) and all_ok
            all_ok = self.dxl_write16(ADDR_CW_LIMIT, 0) and all_ok
            all_ok = self.dxl_write16(ADDR_CCW_LIMIT, 1023) and all_ok
            time.sleep(0.01)
            self.motor_mode = "joint"

        # In joint mode a speed of 0 means "maximum speed", so always
        # leave a real speed in the register.
        speed = max(1, min(1023, int(round(MOTOR_SPEED))))
        all_ok = self.dxl_write16(ADDR_SPEED, speed) and all_ok
        return self.dxl_write8(ADDR_TORQUE, 1) and all_ok

    def init_motor(self):
        """Set up the tool bus and check that the Dynamixel answers.

        Runs once at start-up, before any motion. Leaves the servo in
        position (joint) mode with torque on. Returns False if the motor
        is not ready.
        """
        if not self.set_bus():
            self.print_message(
                "Could not set the tool bus to {} baud in transparent "
                "mode. Clear the controller error in Studio, or set "
                "USE_DYNAMIXEL = False.".format(BUS_BAUD)
            )
            return False

        if self.dxl_request(DXL_PING) is None:
            self.print_message(
                "Dynamixel ID {} is not answering at {} baud. First "
                "time? Run x-armtest4.py and type 'setup'. Otherwise "
                "check the wiring, the 12 V supply and the ID, or set "
                "USE_DYNAMIXEL = False.".format(DXL_ID, BUS_BAUD)
            )
            return False

        self.motor_mode = self.read_motor_mode()
        position = self.get_motor_position()
        if self.motor_mode is None or position is None:
            self.print_message(
                "Dynamixel answered a ping but not the next requests. "
                "Check the wiring and the bus baud rate."
            )
            return False

        if self.motor_mode != "joint":
            self.print_message(
                "Dynamixel is in wheel mode - switching it to position "
                "(joint) mode."
            )

        if not self.use_joint_mode():
            self.print_message(
                "Could not put the Dynamixel in position mode with the "
                "torque on."
            )
            return False

        self.print_message(
            "Dynamixel ready: ID {}, {} baud, position {} ({:.1f} "
            "deg).".format(DXL_ID, BUS_BAUD, position, position * STEP_DEG)
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

        with self.motor_lock:
            position = self.get_motor_position()
            held = position is not None and self.dxl_write16(
                ADDR_GOAL, position
            )

            if not held:
                self.print_message("Could not hold the motor.")

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
        speed = max(1, min(1023, int(round(speed))))
        limited = max(0, min(1023, position))
        if limited != position:
            self.print_message(
                "Motor {}: position {} is outside 0..1023, "
                "using {}.".format(label, position, limited)
            )
        target = int(round(limited))

        self.print_message(
            "Motor to {}: position {}, speed {}".format(
                label, target, speed
            )
        )

        with self.motor_lock:
            # q may have been pressed while waiting for the lock.
            if self.quit_requested:
                return False

            sent = self.dxl_write16(ADDR_SPEED, speed) and self.dxl_write16(
                ADDR_GOAL, target
            )

        if sent and wait:
            sent = self.wait_for_motor(target, label)

        if not sent:
            self.print_message(
                "Motor {}: no answer from the Dynamixel. Stopping the "
                "arm.".format(label)
            )
            self.alive = False
            self.arm.set_state(6)
            return False

        return not self.quit_requested

    def wait_for_motor(self, target, label):
        """Wait until the motor has arrived at target, or stopped.

        Returns False if the servo stops answering, True otherwise
        (including when it is blocked or the timeout passes).
        """
        started = time.time()

        while not self.quit_requested:
            elapsed = time.time() - started

            if elapsed > MOTOR_MOVE_TIMEOUT_SECONDS:
                self.print_message(
                    "Motor {}: still moving after {} s (blocked? "
                    "overloaded?).".format(
                        label, MOTOR_MOVE_TIMEOUT_SECONDS
                    )
                )
                return True

            data = self.dxl_read(ADDR_PRESENT, 11)
            if data is None:
                return False

            position, moving = data[0] | (data[1] << 8), data[10]

            if not moving and abs(position - target) <= 3:
                return True  # arrived

            if not moving and elapsed > 0.5:
                self.print_message(
                    "Motor {}: stopped at {} (blocked or load?).".format(
                        label, position
                    )
                )
                return True

            time.sleep(0.05)

        return True

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
        if not self.move_to_pose(WAYPOINT_3, "waypoint_3"):
            return False
        if not self.move_to_pose(WAYPOINT_4, "waypoint_4"):
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
