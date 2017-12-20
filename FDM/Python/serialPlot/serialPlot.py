import serial  # import Serial Library
import numpy  # Import numpy
import matplotlib.pyplot as plt  # import matplotlib library
from drawnow import *

tempC = []
dutyCycle = []
arduinoData = serial.Serial('com5', 115200)  # Creating our serial object named arduinoData
plt.ion()  # Tell matplotlib you want interactive mode to plot live data
cnt = 0


def makeFig():  # Create a function that makes our desired plot
    # plt.ylim(0, 300)  # Set y min and max values
    plt.title('Temperature Control')  # Plot the title
    plt.grid(True)  # Turn the grid on
    plt.ylabel('Temp C')  # Set ylabels
    plt.plot(tempC, 'ro-', label='Degrees C')  # plot the temperature
    plt.legend(loc='upper left')  # plot the legend
    plt2 = plt.twinx()  # Create a second y axis
    plt.ylim(0, 260)  # Set limits of second y axis- adjust to readings you are getting
    plt2.plot(dutyCycle, 'b^-', label='dutyCycle')  # plot dutyCycle data
    plt2.set_ylabel('Duty Cycle')  # label second y axis
    plt2.ticklabel_format(useOffset=False)  # Force matplotlib to NOT autoscale y axis
    plt2.legend(loc='upper right')  # plot the legend


while True:  # While loop that loops forever
    while (arduinoData.inWaiting() == 0):  # Wait here until there is data
        pass  # do nothing
    arduinoString = arduinoData.readline()  # read the line of text from the serial port
    dataArray = arduinoString.decode().split('\t')  # Split it into an array called dataArray
    temp = float(dataArray[0])  # Convert first element to floating number and put in temp
    dc = float(dataArray[1])  # Convert second element to floating number and put in dc
    tempC.append(temp)  # Build our tempC array by appending temp readings
    dutyCycle.append(dc)  # Building our dutyCycle array by appending dc readings
    drawnow(makeFig)  # Call drawnow to update our live graph
    plt.pause(.000001)  # Pause Briefly. Important to keep drawnow from crashing
    cnt = cnt + 1
    if (cnt > 200):  # If you have 50 or more points, delete the first one from the array
        tempC.pop(0)  # This allows us to just see the last 50 data points
        dutyCycle.pop(0)