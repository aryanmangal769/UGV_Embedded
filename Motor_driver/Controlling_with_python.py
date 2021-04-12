import serial
import time
ser=serial.Serial("COM7",115200)
if ser.isOpen():
    print(ser.name+'is open...')
    
# velocity mode: saved configuration 
       
    ser.write(b'@0gw\r')
    
# velocity at 5%
    ser.write(b'@0sv5\r')
    ser.write(b'@1sv5\r')
#regenerative braking at 0%
    ser.write(b'@0sB0\r')
    ser.write(b'@1sB0\r')

    time.sleep(5)

# velocity at 10%
    ser.write(b'@0sv10\r')
    ser.write(b'@1sv10\r')
    time.sleep(5)

    
# velocity at 0%
    ser.write(b'@0sv0\r')
    ser.write(b'@1sv0\r')
    time.sleep(2)

    ser.write(b'@0sB0\r')
    ser.write(b'@1sB0\r')
#regenerative braking at 10%    
    ser.write(b'@0sB10\r')
    ser.write(b'@1sB10\r')

    time.sleep(5)

    ser.write(b'@0sB0\r')
    ser.write(b'@1sB0\r')
    
    ser.write(b'@0sv0\r')
    ser.write(b'@1sv0\r')
    ser.close()
    print("process terminated")
