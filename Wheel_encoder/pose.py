from __future__ import print_function


import serial

import math

uno = serial.Serial(port='/dev/ttyACM0',baudrate='2000000')

x=0
y=0
theta=0


def read_serial():
    
    global ra
    global ta

    global rb
    global tb

    data=uno.readline()
    print(data)
    sdata=data.split(',')
    print(sdata)

    if len(sdata) == 4:
        try :
            ra = int(sdata[0])
            ta = int(sdata[1])

            rb = int(sdata[2])

            tb =  sdata[3]
            tb = int(tb.replace('\r\n',""))
            
            print("----------Varibles Updated----------")
        except:
            print("Not A Float !!!!")




def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):

    global x
    global y
    global theta

    global ra
    global ta

    global rb
    global tb

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        dist = motor_ticks[0] * ticks_to_mm
        theta = old_pose[2]
        x = old_pose[0] + dist * math.cos(theta)
        y = old_pose[1] + dist * math.sin(theta)
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        # Get old center
        old_theta = old_pose[2]
        old_x = old_pose[0]
        old_y = old_pose[1]


        l = motor_ticks[0] * ticks_to_mm
        r = motor_ticks[1] * ticks_to_mm
        alpha = (r - l) / robot_width
        R = l / alpha

        theta = (old_theta + alpha) % (2*math.pi)
        x = old_x + (R + robot_width/2.0) * (math.sin(theta) - math.sin(old_theta))
        y = old_y + (R + robot_width/2.0) * (-math.cos(theta) + math.cos(old_theta))
        
        return (x, y, theta)



if __name__ == "__main__":


    global ra
    global ta

    global rb
    global tb

    old_ra=0.0
    old_ta=0.0
    old_rb=0.0
    old_tb=0.0

    ticks_to_mm=5
    robot_width=10

    ticks=[0,0]

    while True:
        read_serial()

        ticks[0] = 1000*(ra-old_ra)+(ta-old_ta)
        ticks[1] = 1000*(rb-old_rb)+(tb-old_tb)
        
        pose=[x,y,theta]

        filter_step(pose,ticks,ticks_to_mm,robot_width)

        print(x,y,theta)

        old_ra=ra
        old_ra=ta

        old_ra=rb
        old_ra=tb