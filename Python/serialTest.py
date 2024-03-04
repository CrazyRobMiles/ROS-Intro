# Importing pyserial module
import serial
# Creating a serial object with port name, baud rate, timeout parameters
ser = serial.Serial('dev/ttyACM0', 115200, timeout=1)
data = ser.read(10)
# Printing the data received
print(data)
# Closing the serial port
ser.close()
