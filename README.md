# Kelp Forest Tilt Logger

##### Hardware
- ESP32 Thing Microcontroller
- LSMD91 Inertial Measurement Unit (9DOF)
- SDHC 8GB Card
- 3.7V 10Ah LI Battery

#### Function
This module will be contained in a _buoyant_ capsule underwater that will take various readings from the IMU. It is critical to read the change per second and backsolve for the tilt using integration so that waves can be ignored through numerical model. Tilt readings will be used to solve for ocean current using numerical module.
