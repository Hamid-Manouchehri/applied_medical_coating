#!/usr/bin/env python3
"""Recover the xArm 6 after an emergency stop and send it home.

Run this script (from the PC, or imported into UFactory Studio) and
leave it running. The controller stays powered during an E-stop, so the
script keeps working and waits for a recovery pushbutton:

    1. An E-stop is pressed -> the arm loses power (error C1 or C2).
    2. Fix the cause, check the workspace is clear, then release the
       E-stop (the ROBOT PWR light comes on).
    3. Press the recovery pushbutton (wired to DI3, see below).
    4. The script clears the errors and enables the arm, retrying for
       up to RECOVERY_TIMEOUT_SECONDS because the arm needs a moment to
       power up. Then it moves slowly to HOME_ANGLES and waits for the
       next press.

If it cannot recover, it prints the error/state codes and the manual's
remedy, and nothing moves:

    C1: the E-stop button on the control box is still pushed in.
        Turn it in the direction of its arrow to release it.
    C2: the emergency input is triggered. The two safety inputs EI0 and
        EI1 must both be connected to GND (factory default: a jumper
        from each EI terminal to a GND terminal in the blue "Safety"
        block). If an external E-stop is wired there, release it and
        check that BOTH of its contacts close again.
    C3: the three-state switch E-stop is still pressed.

Wiring of the recovery button (Controller IO terminal block):
    Pushbutton, normally open, between "DI3" and the "GND" terminal
    next to it, both in the green "Digital Inputs" block. The input
    has a weak pull-up, so it reads 1 normally and 0 while the button
    is pressed. Do NOT use the blue "Safety" block (EI0/EI1/SI0/SI1) or
    any 24V terminal. DI3 must stay "General Input" in UFactory Studio
    (Settings - Externals - Controller IO - IO Function).
    DI0, DI1 and DI2 are already used by the peg extraction scripts.

Inputs:
    Robot connection through ROBOT_IP.
    Controller digital input DI3, recovery pushbutton.
    HOME_ANGLES, in degrees.

Outputs:
    Errors cleared, arm enabled, one slow joint-space move to home.

Important:
    The arm sags a little when the E-stop cuts its power, and the home
    move is a straight joint-space move from wherever it ended up. Make
    sure that path is clear before pressing the button - the button
    press is your confirmation. The gripper is not touched.
"""

import time

from xarm.wrapper import XArmAPI


# User Settings
# TODO: IP address of the xArm 6 controller.
ROBOT_IP = "192.168.1.200"

# TODO: DI3 is 11. Feasible range is 8 to 15 for DI0 to DI7.
CONTROLLER_DI_NUMBER = 11

# TODO: Set to 1 if the recovery button reads high when pressed.
BUTTON_VALUE = 0

# TODO: Home pose, (q1, q2, q3, q4, q5, q6) in degrees. Same pose as
# VIA_HOME in XArm6_double_peg_extraction.py. Jog the real arm in
# UFactory Studio and read the angles back to change it.
HOME_ANGLES = [0.0, -16.0, -10.0, 0.0, 27.0, 6.0]

# TODO: Home move joint speed, 0.006 to 229 deg/s. Keep it low: the arm
# starts from wherever the E-stop left it.
HOME_ANGLE_SPEED = 20

# TODO: Home move joint acceleration, 0.57 to 1146 deg/s^2.
HOME_ANGLE_ACCELERATION = 100

# TODO: How long to keep trying to enable the arm after the button is
# pressed, in seconds.
RECOVERY_TIMEOUT_SECONDS = 10.0

# TODO: Pause between enable attempts, in seconds.
RECOVERY_RETRY_SECONDS = 0.5

# TODO: Time between button reads, in seconds. Also the simple debounce.
DIGITAL_READ_INTERVAL_SECONDS = 0.1


# What to do about each E-stop error, from the xArm developer manual.
ESTOP_REMEDIES = {
    1: "C1: release the E-stop button on the control box (turn it in "
       "the direction of its arrow).",
    2: "C2: the emergency input is triggered. Both EI0 and EI1 must be "
       "connected to GND again - release any external E-stop and check "
       "that both of its contacts close, and that the EI jumpers or "
       "wires are in place.",
    3: "C3: release the three-state switch E-stop.",
}


def print_message(message):
    """Print a message with a timestamp."""
    current_time = time.strftime("%Y-%m-%d %H:%M:%S")
    print("[{}] {}".format(current_time, message))


def read_button(arm):
    """Return the recovery button input value (0 or 1), None on error.

    While the controller has an uncleared error (exactly the case after
    an E-stop), the read returns code 1 (or 2 for a warning) as a flag,
    but the input value in the reply is still valid. So 0, 1 and 2 all
    count as a good read; anything else is a real communication error.
    """
    code, read_value = arm.get_cgpio_digital(CONTROLLER_DI_NUMBER)

    if code not in (0, 1, 2):
        print_message("DI read failed, code={}".format(code))
        return None

    return read_value


def read_error_and_state(arm):
    """Return (error_code, warn_code, state) freshly read from the arm."""
    _, error_warn = arm.get_err_warn_code()
    _, state = arm.get_state()
    return error_warn[0], error_warn[1], state


def try_to_enable(arm):
    """Try for a while to clear errors and enable the arm.

    Returns the last (error_code, warn_code, state) it read. The arm is
    ready when error_code is 0 and state is below 4.
    """
    waited_seconds = 0.0

    while waited_seconds < RECOVERY_TIMEOUT_SECONDS:
        arm.clean_warn()
        arm.clean_error()
        arm.motion_enable(True)
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(RECOVERY_RETRY_SECONDS)
        waited_seconds += RECOVERY_RETRY_SECONDS

        error_code, warn_code, state = read_error_and_state(arm)
        if error_code == 0 and state < 4:
            break

    return error_code, warn_code, state


def recover_robot(arm):
    """Clear errors, enable the arm and move it to HOME_ANGLES."""
    print_message("Recovery button pressed. Clearing errors, enabling...")
    error_code, warn_code, state = try_to_enable(arm)

    if error_code != 0 or state >= 4:
        print_message(
            "Cannot recover: error={}, warn={}, state={}.".format(
                error_code, warn_code, state
            )
        )
        if error_code in ESTOP_REMEDIES:
            print_message(ESTOP_REMEDIES[error_code])
        elif error_code != 0:
            print_message(
                "Look up C{} in the xArm developer manual "
                "(error handling).".format(error_code)
            )
        print_message("Fix it and press the recovery button again.")
        return

    print_message("Arm enabled. Moving home: {}".format(HOME_ANGLES))
    code = arm.set_servo_angle(
        angle=HOME_ANGLES,
        speed=HOME_ANGLE_SPEED,
        mvacc=HOME_ANGLE_ACCELERATION,
        wait=True,
        radius=-1.0,
    )

    if code == 0:
        print_message("At home. Waiting for the next recovery press.")
    else:
        print_message("Home move failed, code={}.".format(code))


if __name__ == "__main__":
    read_write_arm = XArmAPI(ROBOT_IP, baud_checkset=False)
    time.sleep(0.5)

    print_message("Ready. Release the E-stop, then press the DI3 button.")
    print_message("Press Ctrl+C to quit.")
    previous_value = read_button(read_write_arm)

    try:
        while read_write_arm.connected:
            button_value = read_button(read_write_arm)
            button_just_pressed = (
                button_value == BUTTON_VALUE
                and previous_value != BUTTON_VALUE
            )

            if button_just_pressed:
                recover_robot(read_write_arm)

            previous_value = button_value
            time.sleep(DIGITAL_READ_INTERVAL_SECONDS)

        print_message("Lost the connection to the controller. Quitting.")
    except KeyboardInterrupt:
        print_message("Stopped by user.")
    finally:
        read_write_arm.disconnect()
