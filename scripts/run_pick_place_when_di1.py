#!/usr/bin/env python3
"""Run pick_place_on_di1.py when xArm controller DI1 is triggered.

This script only watches controller DI1. When DI1 reaches the trigger
value, it starts scripts/pick_place_on_di1.py as a separate Python process.

Inputs:
    Robot connection through ROBOT_IP.
    Controller digital input DI1.

Outputs:
    Starts scripts/pick_place_on_di1.py after DI1 trigger.
"""

import os
import select
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path

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

# TODO: Time after pick/place before accepting another trigger, seconds.
AFTER_RUN_WAIT_SECONDS = 1.0

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
PICK_PLACE_SCRIPT = PROJECT_FOLDER / "scripts" / "pick_place_on_di1.py"


def print_message(*args):
    """Print a simple timestamped message."""
    print(
        "[{}] {}".format(
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time())),
            " ".join(map(str, args)),
        )
    )


def read_di1(read_write_arm):
    """Read controller DI1 value."""
    code, read_di1_value = read_write_arm.get_cgpio_digital(
        CONTROLLER_DI1_NUMBER
    )

    if code != 0:
        print_message("get_cgpio_digital DI1 failed, code={}".format(code))
        return None

    return read_di1_value


def read_keyboard_key():
    """Read one keyboard key without blocking the DI1 loop."""
    read_ready, _, _ = select.select([sys.stdin], [], [], 0)

    if not read_ready:
        return ""

    return sys.stdin.read(1)


def run_pick_place_script():
    """Run pick_place_on_di1.py without waiting for DI1 again."""
    run_environment = os.environ.copy()
    run_environment["RUN_WITHOUT_DI1_WAIT"] = "1"

    print_message("Starting pick_place_on_di1.py")
    subprocess.run(
        [sys.executable, str(PICK_PLACE_SCRIPT)],
        cwd=str(PROJECT_FOLDER),
        env=run_environment,
        check=False,
    )
    print_message("pick_place_on_di1.py finished")


def wait_for_di1_release(read_write_arm):
    """Wait until DI1 is no longer triggered before next trial."""
    print_message("Waiting for DI1 release...")

    while True:
        key = read_keyboard_key()

        if key == "q":
            print_message("q pressed. Quitting.")
            return False

        read_di1_value = read_di1(read_write_arm)

        if read_di1_value is None:
            time.sleep(DI1_READ_INTERVAL_SECONDS)
            continue

        if read_di1_value != DI1_TRIGGER_VALUE:
            print_message("DI1 released. Ready for next trigger.")
            return True

        time.sleep(DI1_READ_INTERVAL_SECONDS)


def main():
    """Watch DI1 and run pick_place_on_di1.py on each new trigger."""
    print_message("xArm-Python-SDK Version:{}".format(version.__version__))
    print_message("Watching DI1 for trigger...")

    read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
    time.sleep(0.5)

    if read_write_arm.warn_code != 0:
        read_write_arm.clean_warn()
    if read_write_arm.error_code != 0:
        read_write_arm.clean_error()

    old_terminal_settings = termios.tcgetattr(sys.stdin)

    try:
        tty.setcbreak(sys.stdin.fileno())
        previous_di1_triggered = False

        while True:
            key = read_keyboard_key()

            if key == "q":
                print_message("q pressed. Quitting.")
                break

            read_di1_value = read_di1(read_write_arm)

            if read_di1_value is None:
                time.sleep(DI1_READ_INTERVAL_SECONDS)
                continue

            read_di1_triggered = read_di1_value == DI1_TRIGGER_VALUE

            if read_di1_triggered and not previous_di1_triggered:
                print_message(
                    "DI1 triggered, value={}".format(read_di1_value)
                )
                run_pick_place_script()
                time.sleep(AFTER_RUN_WAIT_SECONDS)
                if not wait_for_di1_release(read_write_arm):
                    break
                previous_di1_triggered = False
                continue

            previous_di1_triggered = read_di1_triggered
            time.sleep(DI1_READ_INTERVAL_SECONDS)

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings,
        )


if __name__ == "__main__":
    main()
