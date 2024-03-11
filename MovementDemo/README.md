# Robot Movement
In this demo we are going to take control of the robot motors and use them to move the robot around. We are also going to connect a Near Optical Flow sensor to the robot and use that to track robot movement. The aim of the exercise is to allow the user to command the robot to move a distance or direction and for the motion system to be able do this. 

## Connect the Near Optical Flow Sensor

![Flow sensor connected to PICO](images/Flow%20Sensor%20Wiring.jpg)
These are the connections you need to make for the [PAA5100JE Near Optical Flow Sensor](https://shop.pimoroni.com/products/paa5100je-optical-tracking-spi-breakout):

* 3-5v	3V3 Out(36)
* CS	GP17(22)
* SCK	GP18(24)
* MOSI	GP19(25)
* MISO	GP16(21)
* INT	GP21(27)
* GND	GND(16)

The sensor will send movement information out of the PICO serial port which is connected to the Raspberry Pi running the robot application. 

The MicroPython program which controls the PICO sensor can be found in the /Python folder in the file **motionSensor.py**. Make sure that this program runs by storing it in your PICO in a file with the name main.py

You can connect the sensor PICO to any of the usb ports on the Raspberry Pi. If the PICO is not detected the motion node will fail with an error.
## Run the applications
There are three robot nodes you can run to work with the robot. They let you move the robot and see the change in position. You will need to start up three consoles and run each application in a console. The applcatiuons are in the drive package and are called keyboard, motors and motion. Use the commands you saw in the MessageDemo to start and run each. When you press the keyboard move commands you should see the robot move and movement information will be shown in the log.
## **Important**
The motor commands will make the robot motors run. For your first runs you should place the robot on blocks or in the middle of a large area. You can use the space bar in the keyboard controller to stop the robot moving. 