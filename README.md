# My 3D SpaceMouse project

Based on (DIY 3D SpaceMouse by TeachingTech)[https://www.printables.com/model/864950-open-source-spacemouse-space-mushroom-remix] from YouTube 

Code has been changed to run on any ESP device capable of USB-OTG (S2, S3)
Using PlatformIO for crossplatform compilation & custom PID & VID 

# BOM
- ESP32 S2 Mini or S3 Mini (1x)
- JST 2.45 Female Headers or crimping tool with JST 2.45 terminals (1x set or 1x tool and enough terminals)
- Joystick Module (4x)
- Butt connectors, Wago connectors, or other suitable wire-to-wire connection method (As needed, minimum 2)
- 22 AWG or similar stranded wire (As needed)

# Instructions
This project is a modified version of the spacemouse from teaching tech, using a cost-effective ESP32 S2 Mini or S3 Mini. The best part is that you only need to solder the outer pins, which will be utilized in this project.

To connect the joysticks with the ESP32, you'll need JST 2.45 female headers or the ability to crimp your own using a crimp tool. The wiring setup requires three sets of cables: one for power and two for the joystick data (X and Y ADC). The joystick data header is a 1x JST 2.45 x6 header, and the other is a 1x JST 2.45 x2 header.

Here's how to assign the pins:

Joystick 1:

X: PIN 3
Y: PIN 5

Joystick 2:
X: PIN 7
Y: PIN 9

Joystick 3:
X: PIN 11
Y: PIN 12

Joystick 4:
X: PIN 18
Y: PIN 16

Please note that PIN 18 and PIN 16 are the only ADC pins on the other side, so the additional 2x header must be connected to them.
![Copyright goes to the image owner / site owner, sourced from vuknikolic.rs](https://github.com/Xapu1337/NebuJoy/assets/29353794/4c5ee942-5bc0-46fd-8d2f-170f23403d0f)

For power, use a 2x JST 2.45 header in black and red wires. You'll need to create a split connector for the power and ground wires. To do this, use a butt connector, wago, or any other method to wire one cable into four additional ones. Repeat this process for the ground wire.
Once you've crimped the ends, insert them into the 5V and GND pins from the joystick boards. Now, connect the female-to-female header to the ESP32 pins you assigned earlier, as well as the power pins for VBUS and GND.
With everything properly connected, you should be able to flash the ESP32 and see the input register.
