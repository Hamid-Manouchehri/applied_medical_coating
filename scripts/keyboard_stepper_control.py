#!/usr/bin/env python3
"""Control Arduino Mega 2560 stepper motor using keyboard arrow keys.

This script sends serial commands to an Arduino sketch that controls a
TB6600 stepper motor driver.

Inputs:
    Keyboard arrow keys from the PC terminal.
    Serial connection to Arduino Mega 2560.

Outputs:
    Serial commands sent to Arduino:
        f = forward
        b = backward
        s = stop

Keyboard:
    Up arrow: forward
    Down arrow: backward
    Space: stop
    q: quit

Notes: it requires the "serial_keyboard_tb6600_stepper.ino" to be uploaded on arduino.
"""

import select
import sys
import termios
import time
import tty

import serial


# User Settings
# TODO: Change to your Arduino port.
# Common values: /dev/ttyACM0, /dev/ttyUSB0
SERIAL_PORT = "/dev/ttyACM0"

# TODO: Must match SERIAL_BAUD in the Arduino sketch.
SERIAL_BAUD = 115200

# TODO: Time to wait after opening serial, in seconds.
ARDUINO_RESET_WAIT_SECONDS = 2.0


def read_key():
    """Read one keyboard key or escape sequence from terminal."""
    read_ready, _, _ = select.select([sys.stdin], [], [], 0.05)

    if not read_ready:
        return ""

    first_character = sys.stdin.read(1)

    if first_character != "\x1b":
        return first_character

    second_character = sys.stdin.read(1)
    third_character = sys.stdin.read(1)
    return first_character + second_character + third_character


def send_command(read_write_serial, command, previous_command):
    """Send command only when it changed."""
    if command == previous_command:
        return previous_command

    read_write_serial.write(command.encode("ascii"))

    if command == "f":
        print("forward")
    elif command == "b":
        print("backward")
    elif command == "s":
        print("stop")

    return command


def main():
    """Read keyboard arrows and control Arduino stepper motion."""
    print("Opening Arduino serial port:", SERIAL_PORT)
    read_write_serial = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
    time.sleep(ARDUINO_RESET_WAIT_SECONDS)

    print("Keyboard control ready.")
    print("Up arrow = forward")
    print("Down arrow = backward")
    print("Space = stop")
    print("q = quit")

    old_terminal_settings = termios.tcgetattr(sys.stdin)
    previous_command = ""

    try:
        tty.setcbreak(sys.stdin.fileno())
        previous_command = send_command(
            read_write_serial,
            "s",
            previous_command,
        )

        while True:
            key = read_key()

            if key == "\x1b[A":
                previous_command = send_command(
                    read_write_serial,
                    "f",
                    previous_command,
                )
            elif key == "\x1b[B":
                previous_command = send_command(
                    read_write_serial,
                    "b",
                    previous_command,
                )
            elif key == " ":
                previous_command = send_command(
                    read_write_serial,
                    "s",
                    previous_command,
                )
            elif key == "q":
                send_command(read_write_serial, "s", "")
                break

    finally:
        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            old_terminal_settings,
        )
        read_write_serial.close()
        print("Serial closed.")


if __name__ == "__main__":
    main()
