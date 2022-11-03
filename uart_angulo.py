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

# for k in range(26):
#     packet2 = serialInst.read() #readline
#     int_val2 = int.from_bytes(packet2, "big")
#     print(chr(int_val2), end='')

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []
i = 0


# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    packet1 = serialInst.read()
    int_val1 = int.from_bytes(packet1, "big")

    # Add x and y to lists
    xs.append(i)
    ys.append(int_val1)
    i += 1

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('Posición del Servo Motor')
    plt.ylabel('Ángulo de rotación [°]')
    plt.ylim(0, 185)

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=10)
plt.show()

#serialInst.close()
