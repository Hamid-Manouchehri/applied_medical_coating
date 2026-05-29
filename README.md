# xArm 6 Startup Notes

This project contains simple Python scripts for connecting to a UFactory
xArm 6 and running basic motion tests.

## Before Powering Motion

Use this checklist every time before moving the robot:

- Keep the emergency stop within reach.
- Clear the table and robot workspace.
- Check that the gripper and tool are attached firmly.
- Check that the object is not too heavy for the gripper.
- Start with slow motion and high `z` values before moving near the table.
- Do not stand inside the robot motion area while testing.


## Open UFactory Studio

To connect from Ubuntu:

1. Connect the computer to the same network as the xArm controller.
2. If using Ethernet directly, set a manual IPv4 address on the computer.
   Example:

```text
Computer IP: 192.168.1.100
Netmask:     255.255.255.0
Robot IP:    192.168.1.200
```

3. Check that the robot responds:

```bash
ping 192.168.1.200
```

If ping does not work, check the Ethernet cable, network settings, robot IP,
and whether the controller is powered on.

```
sudo ~/softwares/UFactory$ sudo ./UfactoryStudio-Linux-1.0.1.AppImage --no-sandbox
```

enter the follwoing IP address:

192.168.1.200:18333


## Python Setup

If the SDK is not installed, install it first:

```bash
git clone https://github.com/xArm-Developer/xArm-Python-SDK.git
cd xArm-Python-SDK
python3 setup.py install
```

for executing any python script:

```
conda activate ufac_env
```

```
python ~/projects/applied_medical_coating/scripts/script_name.py
```


## Arduino

* open the arduino ide:
```
~/softwares/arduino_ide/arduino-ide_2.3.9_Linux_64bit.AppImage  --no-sandbox
```

* in order to read the output through serial port on your PC: tools -> Serial Monitor

# Dynamic Peg Extraction Software Configuration

## Arduino

Arduino in this setup generates PWM pulse for the motor (stepper motor driver), also it reads from XArm6 controller in such a way if the output (DO0 & DO1) got triggered (through UFactory Studio) it start/stop and change the direction. for that compile and upload `mega2560_tb6600_stepper.ino`.

## Python
A python script takes care of trajectory plannign and dynamic grasping: `dynamic_peg_extraction.py`.


## Push Botton
| pushbutton | XArm6 DI |
| ---------- | --------------------------------- |
| button 1   | DI0 (one cycle of peg extraction) |
| button 2   | DI1 (stop)                        |
| button 3   | DI2 (recovery)                    |
