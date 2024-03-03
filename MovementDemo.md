# Robot Movement
In this demo we are going to take control of the robot motors and use them to move the robot around. We are also going to connect a Near Optical Flow sensor to the robot and use that to track robot movement. The aim of the exercise is to allow the user to command the robot to move a distance or direction and for the motion system to be able do this. 

### Connect the Near Optical Flow Sensor

![Flow sensor connected to PICO](images/Flow%20Sensor%20Wiring.jpg)
These are the connections you need to make for the [PAA5100JE Near Optical Flow Sensor](https://shop.pimoroni.com/products/paa5100je-optical-tracking-spi-breakout):

* 3-5v	3V3 Out(36)
* CS	GP17(22)
* SCK	GP18(24)
* MOSI	GP19(25)
* MISO	GP16(21)
* INT	GP21(27)
* GND	GND(16)


