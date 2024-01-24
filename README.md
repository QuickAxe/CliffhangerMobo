# Code for my race robot for competitive Indian robotics.


This code handles the ESC functions, and includes active stabalisation, using a 6050 MPU.

In my case, I uploaded it to an ardunio pro mini, but any arduino should work, theoretically.
The motor drivers used here are a pair of BTS7960 h-bridge modules, but any dual h-bridge can be used, so long as it can handle the current and voltage requirements.

## Pinout
### Inputs :
direction    ->  D2
throttle     ->  D3
MPU-6050 SDA ->  A4
MPU-6050 SCL ->  A5
Gain Pot     ->  A0
Gain pot is used to set the gain value 

### Outputs:
R_Rpwm -> D5
R_Lpwm -> D6
L_Rpwm -> D9
L_Lpwm -> D10

## References Used
For reading rc signals using hardware interrupts : [kelvineyeone : read pwm decode rc receiver input and apply fail safe](https://projecthub.arduino.cc/kelvineyeone/read-pwm-decode-rc-receiver-input-and-apply-fail-safe-113bac)

For applying corrections using the MPU: [MRSC_Adapter_3Pin_Servo ](https://github.com/TheDIYGuy999/MRSC_Adapter_3Pin_Servo)
