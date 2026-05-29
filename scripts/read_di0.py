#!/usr/bin/env python3
"""Read controller digital input DI0 from a UFactory xArm 6.

This script connects to the robot controller and repeatedly reads DI0.
DI0 is controller GPIO input number 8 in the xArm Python SDK.

Inputs:
    Robot connection through ROBOT_IP.
    Controller digital input DI0.

Outputs:
    Printed DI0 value, where 0 means low and 1 means high.
"""

import time
import traceback

from xarm import version
from xarm.wrapper import XArmAPI


# User Settings
# TODO: Change this to the IP address shown on the xArm controller.
ROBOT_IP = "192.168.1.200"

# TODO: DI0 is 8. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI0_NUMBER = 8

# TODO: Change read interval if you need faster/slower printing.
READ_INTERVAL_SECONDS = 0.2


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


def read_controller_di0(read_write_arm):
    """Read and print controller DI0 until the script is stopped."""
    print_message("Reading controller DI0. Press Ctrl+C to stop.")

    while True:
        code, read_di0_value = read_write_arm.get_cgpio_digital(
            CONTROLLER_DI0_NUMBER
        )

        if code != 0:
            print_message(
                "get_cgpio_digital failed, code={}".format(code)
            )
            break

        print_message("DI0={}".format(read_di0_value))
        time.sleep(READ_INTERVAL_SECONDS)


if __name__ == "__main__":
    print_message("xArm-Python-SDK Version:{}".format(version.__version__))

    read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
    time.sleep(0.5)

    if read_write_arm.warn_code != 0:
        read_write_arm.clean_warn()
    if read_write_arm.error_code != 0:
        read_write_arm.clean_error()

    try:
        read_controller_di0(read_write_arm)
    except KeyboardInterrupt:
        print_message("Stopped by user.")
