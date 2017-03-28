# EATC-120
Emco Turn 120/220 toolchanger driver

This is a driver for the EmcoTurn 120/220 original 8 positions ATC.

This design is derived from the excellent [Toolerator 3000 ](http://bgp.nu/~tom/pub/toolerator3000/) USB controlled driver.
EATC-120 is adding MODBUS capabilities for easy integration with an industrial environment:
* Single 24V power supply with embedded 5V DC/DC converter.
* The “brain” of the module is an Arduino Micro.
* The motor is driven by an LMD18245T providing current control.
* The lock status is automatically detected by measuring the peak current when the motor stalls.
* Everything is reported in MODBUS registers.

Integration with Machinekit/LinuxCNC is easy thanks to “mb2hal” HAL component.

![EATC-120 picture](/pictures/IMG_2226.JPG)
