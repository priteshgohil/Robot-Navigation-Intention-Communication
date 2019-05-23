#import the PySerial library and sleep from the time library
import serial
import os
from time import sleep, time

# scan for ports available, until now the connection is being done automatically
# Working only for one board
# Apparently every board has an unique PID in this case is 239A
# In the case of connect many boards the unique PID shall be previosuly known to connect automatically to that specific board
# "python -m serial.tools.list_ports pid=239A"
# https://pyserial.readthedocs.io/en/latest/tools.html
command=os.popen("python -m serial.tools.list_ports pid=239A").read()

# declare to variables, holding the com port we wish to talk to and the speed
port_i=command.replace(" ","")
port=port_i.replace("\n","")
baud = 9600
lastime = 0
timeout = 1 # 30 seconds timeout

dictionary = {
    "straight": "<RIC##LED:0387:FFFFFF;PAT:1000;BUZ:2000;50;2001#RIC>",
    "reverse": "<RIC##LED:00FC:00EE00;PAT:1001;BUZ:2000;50;2003#RIC>",
    "right": "<RIC##LED:03E0:00FF00;PAT:1002;BUZ:2000;50;2002#RIC>",
    "left": "<RIC##LED:001F:00EEFF;PAT:1003;BUZ:2000;50;2003#RIC>",
    "topRightDiag" : "<RIC##LED:0380:FFFF22;PAT:1004;BUZ:2000;50;2003#RIC>",
    "topLeftDiag" : "<RIC##LED:0007:FFFF22;PAT:1005;BUZ:2000;50;2003#RIC>",
    "bottomRightDiag" : "<RIC##LED:00E0:FFFF22;PAT:1006;BUZ:2000;50;2003#RIC>",
    "bottomLeftDiag" : "<RIC##LED:001C:FFFF22;PAT:1007;BUZ:2000;50;2003#RIC>",
    "rotateCW" : "<RIC##LED:03FF:FF1122;PAT:1008;BUZ:2000;50;2003#RIC>",
    "rotateACW" : "<RIC##LED:03FF:FF1122;PAT:1009;BUZ:2000;50;2003#RIC>",
    "dock" : "<RIC##LED:03FF:FF0000;PAT:1010;BUZ:2000;50;2003#RIC>"
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
print("2: going reverse")
print("3: going right")
print("4: going left")
print("5: going top right diagonal")
print("6: going top left diagonal")
print("7: going bottom right diagonal")
print("8: going bottom left diagonal")
print("9: rotating CW")
print("10: rotating ACW")
print("11: docking load")
print("99: exit")  #Exit command is only for demo
while True:
    value = int(input("Write your command here: "))
    if value == 1:
        command = 'straight'
        arduino.write(dictionary[command].encode())
    elif value == 2:
        command = 'reverse'
        arduino.write(dictionary[command].encode())
    elif value == 3:
        command = 'right'
        arduino.write(dictionary[command].encode())
    elif value == 4:
        command = 'left'
        arduino.write(dictionary[command].encode())
    elif value == 5:
        command = 'topRightDiag'
        arduino.write(dictionary[command].encode())
    elif value == 6:
        command = 'topLeftDiag'
        arduino.write(dictionary[command].encode())
    elif value == 7:
        command = 'bottomRightDiag'
        arduino.write(dictionary[command].encode())
    elif value == 8:
        command = 'bottomLeftDiag'
        arduino.write(dictionary[command].encode())
    elif value == 9:
	command = 'rotateCW'
	arduino.write(dictionary[command].encode())
    elif value == 10:
	command = 'rotateACW'
	arduino.write(dictionary[command].encode())
    elif value == 11:
	command = 'dock'
	arduino.write(dictionary[command].encode())
    elif value == 99:
        print("See you again")
        break
    else:
        print(value)
        print("you did not enter correct choice:")
