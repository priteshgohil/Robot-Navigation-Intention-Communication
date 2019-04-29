#import the PySerial library and sleep from the time library
import serial
from time import sleep, time

# declare to variables, holding the com port we wish to talk to and the speed
port = '/dev/ttyACM1'
baud = 9600
lastime = 0
timeout = 1 # 30 seconds timeout

dictionary = {
    "straight": "<RIC##LED:00F0:FF0000;PAT:1000;BUZ:2000;50;2001#RIC>",
    "right": "<RIC##LED:001F:00FF00;PAT:1001;BUZ:2000;50;2002#RIC>",
    "left": "<RIC##LED:02E0:0000FF;PAT:1002;BUZ:2000;50;2003#RIC>"
}

#Opening of the serial port
try:
    arduino = serial.Serial(port,baud,timeout=1)
except:
    print('Please check the port')
    
#Initialising variables
serial.Serial()
rawdata=[]
count=0

print("Enter following command for to see the robot actions:")
print("1: going straight")
print("2: going right")
print("3: going left")
print("4: exit")  #Exit command is only for demo
while True:
    value = int(input("Write your command here: "))
    if value == 1:
        command = 'straight'
        arduino.write(dictionary[command])
    elif value == 2:
        command = 'right'
        arduino.write(dictionary[command])
    elif value == 3:
        command = 'left'
        arduino.write(dictionary[command])
    elif value == 4: 
        print("See you again")
        break
    else:
        print(value)
        print("you did not enter correct choice:")
