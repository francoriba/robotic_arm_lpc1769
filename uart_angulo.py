from curses import baudrate
import serial.tools.list_ports
import time
import keyboard 
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

ports = serial.tools.list_ports.comports()
portList = []

print("\n LIST OF ALL PORTS:")
for onePort in ports:
    portList.append(str(onePort))
    print("\t"+ str(onePort)+ "\n")

val = input("Select Port Number (ttsyUSB on Linux or COM on Windows): ")

for x in range(0, len(portList)):
    if portList[x].startswith("COM" + str(val)):
        portVar = "COM" + str(val)
        print("\t"+"Selected port is: " + portList[x])
    elif portList[x].startswith("/dev/ttyUSB" + str(val)):
        portVar = "/dev/ttyUSB" + str(val)
        print("\t"+"Selected port is: " + portList[x])

serialInst = serial.Serial()
serialInst.baudrate = 9600
serialInst.port = portVar

serialInst.open()

cont = 0


plt.ion()
fig = plt.figure()

x = list()
y = list()
i = 0




while True:   
    cont += 1
    if cont == 1: 
        packet1 = serialInst.read() #readline
        int_val1 = int.from_bytes(packet1, "big")
        print("Shoulder joint:\t", int_val1)
        x.append(i)
        y.append(int_val1)
        plt.scatter(i, int(int_val1))
        i += 1
        plt.grid()
        plt.show()
        plt.pause(0.000001)

    elif cont == 2:
        packet2 = serialInst.read() #readline
        int_val2 = int.from_bytes(packet2, "big")
        print("Elbow joint:\t", int_val2)

    elif cont == 3:
        packet3 = serialInst.read() #readline
        int_val3 = int.from_bytes(packet3, "big")
        print("Wrist joint 1:\t", int_val3)

    elif cont == 4:
        packet4 = serialInst.read() #readline
        int_val4 = int.from_bytes(packet4, "big")
        print("Wrist joint 2:\t", int_val4)

    elif cont == 5:
        packet5 = serialInst.read() #readline
        int_val5 = int.from_bytes(packet5, "big")
        print("Gripper joint:\t", int_val5)
        cont = 0



    #time.sleep(0.1)



serialInst.close()