# Arduino Mega 2560 And TB6600 Stepper Driver

This folder contains simple Arduino programs for driving a stepper motor
with an Arduino Mega 2560 and a TB6600 stepper motor driver.

PC-upload test sketch:

```text
arduino/mega2560_tb6600_stepper/mega2560_tb6600_stepper.ino
```

Robot DO0-controlled sketch:

```text
arduino/mega2560_tb6600_stepper_do0/mega2560_tb6600_stepper_do0.ino
```

## Hardware Needed

- Arduino Mega 2560
- TB6600 stepper motor driver
- Stepper motor
- External motor power supply for the TB6600
- Jumper wires
- USB cable for Arduino programming

## Important Power Notes

- Do not power the stepper motor from the Arduino.
- The TB6600 motor power supply must match the motor and driver rating.
- The Arduino sends only control signals: `STEP`, `DIR`, and `ENABLE`.
- Turn off motor power before changing motor wiring.
- Keep the motor power supply ground and Arduino ground connected together.

## Arduino Pin Settings

The sketch currently uses:

```cpp
const int STEP_PIN   = 3;
const int DIR_PIN    = 2;
const int ENABLE_PIN = 4;
```

You can change these in the User Settings section of the sketch.

## Signal Wiring

Use this common-cathode wiring style:

| Arduino Mega 2560 | TB6600 Terminal |
| ----------------- | --------------- |
| Pin 3             | PUL+            |
| Pin 2             | DIR+            |
| Pin 4             | ENA+            |
| GND               | PUL-            |
| GND               | DIR-            |
| GND               | ENA-            |

With this wiring, Arduino `HIGH` means the signal is active at the TB6600
input. The sketch assumes `ENABLE_ACTIVE_LOW = true`, which is common for
many TB6600 modules. If the motor does not energize, try:

```cpp
const bool ENABLE_ACTIVE_LOW = false;
```

## Robot DO0 Wiring For DO0 Sketch

This section applies only to:

```text
arduino/mega2560_tb6600_stepper_do0/mega2560_tb6600_stepper_do0.ino
```

Connect the robot `DO0` signal to the Arduino input pin:

| Robot / Controller | Arduino Mega 2560 |
| ------------------ | ----------------- |
| DO0 signal          | Pin 8             |
| DO0 / signal ground | GND               |

The DO0 sketch assumes `DO0` is active when Arduino pin 8 reads `HIGH`:

```cpp
const bool DO0_ACTIVE_HIGH = true;
```

If your wiring gives `LOW` when `DO0` is active, change it to:

```cpp
const bool DO0_ACTIVE_HIGH = false;
```

Important: connect the robot signal ground to Arduino `GND`, unless your
interface uses an isolated relay/contact module.

## Motor Wiring

Connect the stepper motor coils to the TB6600 motor outputs:

| Stepper Coil | TB6600 Terminal |
| ------------ | --------------- |
| Coil A wire 1 | A+             |
| Coil A wire 2 | A-             |
| Coil B wire 1 | B+             |
| Coil B wire 2 | B-             |

If the motor vibrates but does not rotate, the coil pairs are probably
incorrect. Use a multimeter to identify the two separate coil pairs.

## Motor Power Wiring

Connect the external motor power supply to the TB6600:

| Power Supply | TB6600 Terminal |
| ------------ | --------------- |
| Positive     | VCC / V+        |
| Negative     | GND / V-        |

The exact terminal names can change between TB6600 modules. Check the label
on your driver.

## TB6600 DIP Switches

Set the TB6600 current limit based on your motor rated phase current.

Start conservative:

- Use a current setting at or below the motor rated current.
- Use a simple microstep setting first, such as 1/8 or 1/16.
- Record the DIP switch setting in your lab notes.

The PC-upload test sketch uses:

```cpp
const long STEPS_PER_MOVE = 1600;
```

The real rotation angle depends on:

```text
motor step angle
TB6600 microstep setting
STEPS_PER_MOVE
```

Example for a 1.8 degree motor:

```text
200 full steps = 1 motor revolution
1/8 microstepping = 1600 input pulses per revolution
```

In the DO0 sketch the motor keeps stepping while `DO0` is active.

## Upload The Sketch

1. Open Arduino IDE.
2. Open:

```text
arduino/mega2560_tb6600_stepper/mega2560_tb6600_stepper.ino
```

3. Select board:

```text
Arduino Mega or Mega 2560
```

4. Select the correct port.
5. Click Upload.

## First Test

Before applying motor power:

1. Check all signal wiring.
2. Check motor coil wiring.
3. Check TB6600 current DIP switches.
4. Keep the motor mechanically unloaded.
5. Keep one hand near the motor power switch.

Then:

1. Connect Arduino USB.
2. Upload the sketch.
3. Turn on motor power.
4. For the PC-upload test sketch, the motor should rotate forward, pause,
   rotate backward, and repeat.
5. For the DO0 sketch, activate robot `DO0`.
6. The motor should rotate while `DO0` is active.
7. Deactivate robot `DO0`.
8. The motor should stop.

## Adjust Speed

The sketch controls speed using:

```cpp
const unsigned int STEP_DELAY_US = 800;
```

Larger value:

```text
slower motor
```

Smaller value:

```text
faster motor
```

For first tests, use a larger delay such as `1000` or `1500`.

## Adjust Distance

The PC-upload test sketch controls move distance using:

```cpp
const long STEPS_PER_MOVE = 1600;
```

Increase this value for more rotation. Decrease it for less rotation.

## Troubleshooting

If the motor does not move:

- Check that motor power is on.
- Check that Arduino GND is connected to TB6600 signal GND.
- Check that `PUL+`, `DIR+`, and `ENA+` are connected correctly.
- Check that robot `DO0` is connected to Arduino pin 8.
- Check that robot signal ground is connected to Arduino `GND`.
- Try changing `DO0_ACTIVE_HIGH`.
- Try changing `ENABLE_ACTIVE_LOW`.
- Try a slower `STEP_DELAY_US`, such as `1500`.
- Check the TB6600 DIP switch current setting.

If the motor only vibrates:

- Check the stepper coil pairs.
- Reduce speed by increasing `STEP_DELAY_US`.
- Reduce load on the motor.
- Check that the motor power supply current is sufficient.

If the motor turns the wrong direction:

- Swap `A+` and `A-`, or change the `DIR_PIN` logic in the sketch.
