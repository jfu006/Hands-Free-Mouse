# Hands-Free-Mouse

Wireless Hands-Free Mouse (Motion Controller Device)

Overview

Wireless Hands-Free Mouse is a device which allows the user to control their computer screen via gestures and motions. The device considers speed and direction to navigate through a computer. It creates hands-free control over the cursor of a computer.


The device works both attached to a battery or plugged into a 5V power supply. The battery is needed to make the device completely wireless. Also, an elastic strap can be attached to the device into a wearable, making it “hands-free”. In this case, the device does not have a strap.


User Guide

Set Up:

1. After following the Component and Pinout Guide, power the device with 5V. Make sure the device is powered the entire time.

2. Make sure the switch on the device is OFF.

3. Program the ATmega1284 using AVR Studio and its affiliated code.

4. On the Mac OS terminal, on the laptop which the cursor will be controlled 

-Turn on Bluetooth and connect to the Bluetooth module on the device.

-Compile click.m using “gcc -o click click.m -framework ApplicationServices -framework Foundation”.

-Make sure “pyserial” is installed beforehand.

-Run listener.py using “python listener.py”.

5. Wait for the light on the Bluetooth module to stop blinking.

6. Turn the switch on the device ON.

7. The terminal will output the cursor’s current coordinates. Movement to the device will now control the movement of the cursor.


Technologies/Components

Hardware:

-ATmega1284

-Adafruit BNO055 Absolute Orientation Sensor

-Wireless Bluetooth module

-Switch

-Resistors (2 x 4.7k Ohms, 1 x 1k Ohms)

Software: https://github.com/jfu006/Hands-Free-Mouse 

-AVR Studio 7 (main.cpp, usart_ATmega1284.h, i2c.c)

-Mac OS (click.m, listener.py)
