# Visualization of CubeSat Orientation using vPython
"""
Summary:
    This script reads quaternion data from a serial port and visualizes the orientation of a CubeSat in 3D using the vpython library.

Purpose:
    This script is used to visualize the orientation of a CubeSat in 3D using the vpython library. The orientation data is read from
    a serial port in the form of quaternions and converted to Euler angles (roll, pitch, and yaw).
    The CubeSat orientation is then updated in the 3D visualization based on the Euler angles.

Inputs:
    Quaternion data from a serial port.

    q0 - Quaternion scalar component.
    q1 - Quaternion x-component.
    q2 - Quaternion y-component.
    q3 - Quaternion z-component.

Outputs:
    3D visualization of the CubeSat orientation.
"""


from vpython import *   # Importing vpython for 3D visualization
from time import sleep  # Importing sleep function from time module
import numpy as np      # Importing numpy for numerical operations
import math             # Importing math for mathematical operations
import serial           # Importing serial for serial communication

# Initialize serial communication with the specified port and baud rate
ad = serial.Serial('com3', 115200)
sleep(1)  # Wait for 1 second to establish the connection

# Set up the scene for visualization
scene.range = 5                     # Set the range of the scene
scene.background = color.black      # Set the background color to black
toRad = 2 * np.pi / 360             # Conversion factor from degrees to radians
toDeg = 1 / toRad                   # Conversion factor from radians to degrees
scene.forward = vector(-1, -1, -1)  # Set the initial view direction

# Set the dimensions of the scene window
scene.width = 1200
scene.height = 1080

# Create arrows to represent the coordinate axes
xarrow = arrow(length=2, shaftwidth=0.1, color=color.red, axis=vector(1, 0, 0))
yarrow = arrow(length=2, shaftwidth=0.1, color=color.green, axis=vector(0, 1, 0))
zarrow = arrow(length=4, shaftwidth=0.1, color=color.blue, axis=vector(0, 0, 1))

# Create arrows to represent the orientation of the CubeSat
frontArrow = arrow(length=4, shaftwidth=0.1, color=color.purple, axis=vector(1, 0, 0))
upArrow = arrow(length=1, shaftwidth=0.1, color=color.magenta, axis=vector(0, 1, 0))
sideArrow = arrow(length=2, shaftwidth=0.1, color=color.orange, axis=vector(0, 0, 1))

# Dimensions for the CubeSat components
cubeSatLength = 3   # 30 cm
cubeSatWidth = 2    # 20 cm
cubeSatHeight = 0.6 # 6 cm

# Define the components of the CubeSat
cubeSatBody = box(length=cubeSatLength, width=cubeSatWidth, height=cubeSatHeight, color=color.gray(0.5))

# Create a compound object for the CubeSat
cubeSat = compound([cubeSatBody])

# Main loop to read data from the serial port and update the visualization
while True:
    try:
        # Wait until there is data available in the serial buffer
        while ad.inWaiting() == 0:
            pass
        
        # Read a line of data from the serial port
        dataPacket = ad.readline()
        dataPacket = str(dataPacket, 'utf-8')   # Convert the byte data to string
        splitPacket = dataPacket.split(",")     # Split the data by commas
        
        # Extract quaternion values from the data packet
        q0 = float(splitPacket[0])
        q1 = float(splitPacket[1])
        q2 = float(splitPacket[2])
        q3 = float(splitPacket[3])
        
        # Calculate roll, pitch, and yaw from the quaternion values
        roll = -math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        pitch = math.asin(2 * (q0 * q2 - q3 * q1))
        yaw = -math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3)) - np.pi / 2
        
        rate(50)  # Limit the update rate to 50 frames per second
        
        # Calculate the direction vectors for the CubeSat orientation
        k = vector(cos(yaw) * cos(pitch), sin(pitch), sin(yaw) * cos(pitch))
        y = vector(0, 1, 0)
        s = cross(k, y)
        v = cross(s, k)
        vrot = v * cos(roll) + cross(k, v) * sin(roll)
        
        # Update the orientation of the CubeSat and the arrows
        cubeSat.axis = k
        cubeSat.up = vrot
        frontArrow.axis = k
        upArrow.axis = vrot
        sideArrow.axis
