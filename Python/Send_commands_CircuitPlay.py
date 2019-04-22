# -*- coding: utf-8 -*-
import serial, time
# import tkinter
import os

"""Opening of the serial port"""
try:
    ard = serial.Serial("/dev/ttyACM1",9600,timeout=1)
except:
    print('Please check the port')

"""Initialising variables"""
rawdata=[]
count=0
keep_sending_data=True
"""Receiving data and storing it in a list"""
while keep_sending_data:
    value = int(input("Insert the data to be sent 1:Goto 2:Dock 3:exit"))

    if value==1:
        print("Goto:blue:5")
        temp= "Goto:blue:5\r\n"
        ard.write(temp.encode())
    elif value==2:
        print("Dock:blue:5")
        temp= "Dock:blue:5\r\n"
        ard.write(temp.encode())
    else:
        keep_sending_data=False
    # rawdata.append(str(ard.readline()))
    # print(str(ard.readline()))
    # count+=1
    # if (count>10):
    #     ard.write(count)
    #     count=0


def clean(L):#L is a list
    newl=[]#initialising the new list
    for i in range(len(L)):
        temp=L[i][2:]
        newl.append(temp[:-5])
    return newl

cleandata=clean(rawdata)

def write(L):
    file=open("data.txt",mode='w')
    for i in range(len(L)):
        file.write(L[i]+'\n')
    file.close()

write(cleandata)
