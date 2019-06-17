import serial
import os
from time import sleep, time

class SerialInterface:
    def __init__(self, baud, timeout, pid):
        self.baud = baud
        self.timeout = timeout
        self.pid = pid

    def open_port(self):
        command=os.popen("python -m serial.tools.list_ports pid=239A").read()
        port_i=command.replace(" ","")
        port=port_i.replace("\n","")
        #Opening of the serial port
        try:
            self.arduino = serial.Serial(port,self.baud,timeout=self.timeout)
        except:
            print('Please check the port')

    def send_command(self,message_to_send):
        print("sending cmd")
        self.arduino.flushInput()
        self.arduino.write(message_to_send.encode())
