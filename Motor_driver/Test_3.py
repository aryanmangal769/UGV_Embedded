import serial
import struct
driver=serial.Serial('com3',9600)
output="@alpha1"
driver.write(output.encode())
while 1:
    file=driver.read()
    book+=file +""
    print(book)
