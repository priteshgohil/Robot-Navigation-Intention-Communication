"""
This module contains a component that communicates with
the particular arduino board and sends the message framework via
serial port with specified baudrate, timeout and board pid.
"""
import serial
import os
from time import sleep, time

class SerialInterface:
    """
    Initialize class with baudrate, timeout, and pid of arduino board
    which is fixed in our case pid = 239A.
    """
    def __init__(self, baud, timeout, pid):
        self.baud = baud
        self.timeout = timeout
        self.pid = pid
    """
    Detect the arduino board and initialize the serial module from pyserial

    """
    def open_port(self):
        command=os.popen("python -m serial.tools.list_ports pid=239A").read()
        port_i=command.replace(" ","")
        port=port_i.replace("\n","")
        #Opening of the serial port
        try:
            self.arduino = serial.Serial(port,self.baud,timeout=self.timeout)
        except:
            print('Please check the port')

    """
    Send the string message framework to arduino

    """
    def send_command(self,message_to_send):
        print("sending cmd")
        self.arduino.flushInput()
        self.arduino.write(message_to_send.encode())
