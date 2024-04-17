# Motor Controller Setup

1. Open the VESC tool
2. Make sure the micro-USB to USB cable is connected to the motor controller 
and the computer and hit `AutoConnect` on the `Welcome & Wizards` page of the 
VESC tool
3. Click `Setup Motors FOC` on `Welcome & Wizards` and select `Yes` to load 
default parameters.
4. For `USAGE` select `Generic` and then hit next.
5. For `MOTOR` select `Small Inrunner` and then set the `Motor Poles` to 4 
and then hit next.
6. For `BATTERY` select the `LIION` option, 3 cell series, and 5.0 Ah and hit 
next.
7. For `SETUP` check `Direct Drive`, set the wheel diameter to 110.0 mm, and 
change the number of poles to 4.
8. Hit `RUN DETECTION`.
9. After it is complete, you should not see `SENSORLESS` in the detection 
result.
