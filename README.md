# ProjectX Robot

Code and hardware files for an educational robot using a Seeed XIAO ESP32-C3, MPU6050, HC-SR04, DRV8833 motor driver and two N20 motors.

## Folder guide

| Folder | Contents |
| --- | --- |
| [Firmware/Robot_ArduinoIDE](Firmware/Robot_ArduinoIDE/) | Arduino IDE version of the robot controller, with bundled libraries. |
| [Firmware/Main_Code_PlatformIO](Firmware/Main_Code_PlatformIO/) | PlatformIO version of the main robot controller. |
| [Firmware/PIDtest_Only_PlatformIO](Firmware/PIDtest_Only_PlatformIO/) | Standalone PID/self-balancing test project for PlatformIO. |
| [Firmware/PID_test](Firmware/PID_test/) | Standalone Arduino PID test sketch. |
| [Hardware](Hardware/) | Robot hardware design files, including `ProjectX_SelfBalance_PCB`. |

The main firmware provides Wi-Fi remote driving and ultrasonic obstacle avoidance, with attitude correction and calibration controls on the webpage. Both main firmware versions include a saved 20 cm ultrasonic reference calibration.

## Getting started

Open `Firmware/Robot_ArduinoIDE/Robot_ArduinoIDE.ino` in Arduino IDE, select **XIAO_ESP32C3**, and enable **USB CDC On Boot**. Editable settings are in `robot_config.h`. After uploading, connect to the robot's Wi-Fi and open **http://192.168.4.1/** (default address).

See the [Arduino README](Firmware/Robot_ArduinoIDE/README.md) for setup, wiring and calibration instructions. Keep the standalone PID tests separate from the main sketch.
