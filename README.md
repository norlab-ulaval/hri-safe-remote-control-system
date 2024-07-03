# FORT VSC ROS driver

This repo contains the ROS driver for the [FORT Vehicle Safety Controller (VSC)](https://www.fortrobotics.com/vehicle-safety-controller).
The VCS is also known as the Safe Remote Control System (SRCS) of [Humanistic Robotics](https://humanisticrobotics.com).

## Repo Contents

* C Drivers: [/drivers](/drivers)
* ROS Drivers: [/hri_safety_sense](/hri_safety_sense)

## Tutorial Instructions

Tutorials are provided in [Tutorials](/drivers/C/tutorials) :

* `vsc_tutorial_1`: A message of type UserHeartbeatMsgType containing the EStopStatus is sent at a rate of 20Hz to the VSC. This is a basic example where the joystick and button values can be seen as received by the VSC.
* `vsc_tutorial_2`: This tutorial shows how to switch the SRC into custom text display mode. This example is based on the first tutorial and expands it by settings three default display lines, and constantly updating the fourth.
* `vsc_tutorial_3`: The third tutorial shows how to switch the SRC into the text/value display mode. The example shows how to constantly update user feedback values on the SRC display.
* `vsc_tutorial_4`: The fourth example shows how to switch the SRC into the user value displa mode. The example shows how to constantly update user feedback values and also send motor control commands to the SRC.

To run vsc_tutorial_1, ensure that the SRC and VSC are on the same network.

Under the cloned directory `/driver/C` execute the following command in your terminal:

```sh
./tutorials/vsc_tutorial_1 /ttyACM* 115200
```

where `*` replaces the serial port being used for the VSC.

Note: This example assumes the VSC is connected via USB to a user computer. Ports used can be found under the `/dev` directory in Unix-based systems. Usually, `ttyACM0/1`, `tty.usbserial`.
