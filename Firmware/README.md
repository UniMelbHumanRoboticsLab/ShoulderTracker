## Shoulder Tracker Firmware
Arduino code intended to run on a Bluno board (either Beetle BLE or Bluno nano) connected to an IMU through I2C (either a Pololu MinIMU-9 v3 or Pololu AltIMU-10 v5).

The code:
- perform measurements of the posture or movement using the IMU
- compute an adaptive threshold
- provide feedback when the measured posture/movement is above threshold
- manage communication (for control and logging) with the host software running on the computer


See [here](https://wiki.dfrobot.com/Bluno_SKU_DFR0267#target_4) to configure the Bluno dongle in CENTRAL mode. In short:
```
+++
AT+SETTING=DEFCENTRAL
AT+SETTING=?
AT+EXIT
````
(with CR+LF). 
