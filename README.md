# ICM42605 combo accel gyro

![](https://user-images.githubusercontent.com/6698410/61191936-f6426e80-a664-11e9-95fd-52ed28a7e50f.jpg)

Basic Arduino sketch for TDK's (Invensense's) [ICM42605](https://www.invensense.com/products/motion-tracking/6-axis/icm-42605/) combination 6 DoF accel/gyro. Configure sensor full scale and sample rates, configure data ready interrupt, basic offset calibration, read and scale sensor data, combine with LIS2MDL magnetometer data to get quaternions and absolute orientation using Madgwick's open-source fusion algorithm. Include LPS22HB barometer. All running on an STM32L476RE Dragonfly deveopment board with fusion rate at 20x the gyro sample rate.

Lot's of possibilities with this sensor including embedded functions such as tilt detection, step counting, significant motion as well as wake on motion, wake from sleep, etc.
