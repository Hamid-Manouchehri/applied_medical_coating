#!/usr/bin/env python3
"""Simple pick-and-place script for UFactory xArm 6.

This script connects to the xArm, moves to a safe start pose, opens the
gripper, moves to the pick position, closes the gripper, moves to the place
position, opens the gripper, and returns upward.

Inputs:
    Robot connection through ROBOT_IP.
    Pick/place Cartesian poses in millimeters and degrees.

Outputs:
    Robot motion commands sent to the xArm 6.
"""

import time
import traceback

from xarm import version
from xarm.wrapper import XArmAPI


# User Settings
# TODO: Change this to the IP address shown on the xArm controller.
ROBOT_IP = "192.168.1.200"

# TODO: Change these poses after checking the real robot workspace.
SAFE_START_ANGLES = [105.0, -11.1, -56.4, 0.0, 67.5, 24.5]

# TODO: Change pick/place positions for your table and object.
PICK_ABOVE_POSE = [-79.8, 412.1, 400.0, 180.0, 0.0, 76.5]
PICK_POSE = [-79.8, 412.1, 200.0, 180.0, 0.0, 76.5]
PLACE_ABOVE_POSE = [522.0, -7.0, 400.0, 180.0, 0.0, 4.6]
PLACE_POSE = [522.0, -7.0, 300.0, 180.0, 0.0, 4.6]

# TODO: Change gripper values if your object needs different opening.
GRIPPER_OPEN_POSITION = 800
GRIPPER_CLOSED_POSITION = 0

TCP_SPEED = 200
TCP_ACCELERATION = 5000
ANGLE_SPEED = 20
ANGLE_ACCELERATION = 500
GRIPPER_SPEED = 5000

# xArm gripper payload only. Update if you know your real tool payload.
GRIPPER_MASS_KG = 0.82
GRIPPER_CENTER_OF_MASS_MM = [0, 0, 48]


class SimplePickPlaceRobot:
    """Small readable wrapper for a simple xArm 6 pick-and-place program."""

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

    def run(self):
        """Run one pick-and-place cycle."""
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

            if not self.move_to_pose(PICK_ABOVE_POSE, "move_pick_above"):
                return
            if not self.move_to_pose(PICK_POSE, "move_pick"):
                return

            if not self.set_gripper_closed():
                return

            if not self.move_to_pose(PICK_ABOVE_POSE, "move_up_from_pick"):
                return
            if not self.move_to_pose(PLACE_ABOVE_POSE, "move_place_above"):
                return
            if not self.move_to_pose(PLACE_POSE, "move_place"):
                return

            if not self.set_gripper_open():
                return

            code = self.arm.set_tcp_load(
                GRIPPER_MASS_KG,
                GRIPPER_CENTER_OF_MASS_MM,
            )
            if not self.check_code(code, "set_tcp_load_gripper"):
                return

            self.move_to_pose(PLACE_ABOVE_POSE, "move_up_from_place")

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
    SimplePickPlaceRobot.print_message(
        "xArm-Python-SDK Version:{}".format(version.__version__)
    )
    read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
    time.sleep(0.5)

    robot = SimplePickPlaceRobot(read_write_arm)
    robot.run()
