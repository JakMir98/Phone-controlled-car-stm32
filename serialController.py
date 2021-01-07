import serial
import time

uart = serial.Serial("COM7", 9600)
time.sleep(3)
uart.write(bytearray('o','ascii'));
time.sleep(.3)
uart.write(bytearray('f','ascii'));
time.sleep(1)
uart.write(bytearray('o','ascii'));
time.sleep(.3)
for i in range(5):
    uart.write(bytearray('u','ascii'));
    time.sleep(.2)

uart.write(bytearray('n','ascii'));
time.sleep(.3)

for i in range(5):
    uart.write(bytearray('r','ascii'));
    time.sleep(.5)

uart.write(bytearray('n','ascii'));
time.sleep(.3)

for i in range(5):
    uart.write(bytearray('l','ascii'));
    time.sleep(.5)

uart.write(bytearray('n','ascii'));
time.sleep(.3)

for i in range(5):
    uart.write(bytearray('d','ascii'));
    time.sleep(.5)

uart.write(bytearray('n','ascii'));
time.sleep(.3)


uart.write(bytearray('n','ascii'));
time.sleep(.3)
uart.write(bytearray('h','ascii'));
time.sleep(.3)
uart.write(bytearray('n','ascii'));
