# Quadruped-Robot
This repository is based on college project. Work on single STM32F103CBT6 microcontroller, MPU6050 and PCA9685 driver board.

Due to lack of resources the project was temporarily discounted.

Work done so for:
Inverse Kinematics implementation.
Gait planning.
Balancing PID control.

Cutrent issues:
MG996r servos are used, which can't handle robot weight.
MPU6050 was not calibrated.
Commands for operation was sent via USART using FT232 USB to TTL module.

References:
https://spotmicroai.readthedocs.io/en/latest/
https://github.com/mike4192/spotMicro
https://gitlab.com/public-open-source/spotmicroai
https://novaspotmicro.com/
https://github.com/leech001/MPU6050
